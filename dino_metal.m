// dino_metal.m - Metal-accelerated viewer for the Dino-Lite AM411T USB microscope
// (AnMo a168:0615, Sonix SN9C201 bridge + Micron MT9M111 sensor).
//
// libusb engine streams RAW Bayer SXGA (1280x1023) over isochronous EP 0x81; a Metal
// fragment shader debayers (BGGR, block-chroma + full-res luma) and applies white balance,
// sRGB gamma and contrast; MTKView presents it. The MT9M111 runs its own hardware
// auto-exposure/AGC. White-patch AWB on the CPU converges then locks. Capture saves a PNG.
//
// Build: see Makefile (or `make`). Driver protocol ported from the Linux gspca sn9c20x driver.
#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <ImageIO/ImageIO.h>
#include <libusb-1.0/libusb.h>
#include <pthread.h>

#define VID 0xA168
#define PID 0x0615
#define IFACE 0
#define ALT 8
#define EP_ISO 0x81
#define W 1280            // sensor window width
#define H 1024            // window height (for the bridge window registers)
#define CH 1023           // rows the bridge actually emits at SXGA (one short of 1024)
#define FRAME_BYTES (W*CH)
#define I2C_ADDR 0x5d
#define I2C_INTF 0x81
// YUV hardware-demosaic mode (SN9C201 demosaics in silicon; capped at <=640x480).
#define YUV_W 640
#define YUV_WIN_H 480              // window registers use 480
#define YUV_H 472                  // ...but the bridge emits 472 rows (like SXGA's 1023)
#define YUV_FRAME_BYTES (YUV_W*YUV_H*3/2)   // custom I420 macroblock stream = 453120
#define YUV_RGBA_BYTES (YUV_W*YUV_H*4)

static libusb_device_handle *dev; static libusb_context *ctx;
static int g_err=0, g_run=1; static pthread_t g_engine;
static volatile int g_devlost=0;        // set when the device disappears (unplug); triggers reconnect
static volatile unsigned g_frames=0;    // monotonic frame counter (engine watchdog + UI)
static int g_inflight=0;                 // iso transfers libusb still owns (engine thread only)
static volatile int g_led=0x20, g_led_applied=-1;  // 0x20=on, 0x00=off (single GPIO LED, no brightness)
static volatile int g_capture_req=0;
static volatile int g_gray=0;
static volatile int g_phase=3;        // Bayer phase 0=GBRG 1=GRBG 2=RGGB 3=BGGR
static volatile float g_contrast=1.15f;  // match vendor DinoCapture (Contrast 1.15); 1.8 over-amplifies edge fringe on sharp halftone prints
static volatile float g_gR=1.0f, g_gG=1.0f, g_gB=1.0f;
static pthread_mutex_t lock=PTHREAD_MUTEX_INITIALIZER;
static uint8_t shared_raw[FRAME_BYTES]; static int shared_new=0;
static volatile int g_awb_n=14;   // AWB converge-then-hold counter (converges over N frames, then locks)
// ---- YUV mode bus ----
static volatile int g_mode=0, g_mode_applied=-1;   // 0=SXGA Bayer  1=YUV 640 hardware-demosaic
static volatile int g_exp=0x180, g_exp_applied=-1; // MT9M111 shutter (0x09) - manual Luma in YUV mode
static volatile int g_sat=160;                     // bridge cmatrix saturation (set once at YUV start)
static volatile int g_red=0x1f, g_blue=0x1f;       // bridge red/blue gain (0x118c/0x118f)
static volatile float g_cscale=2.2f;               // YUV baseline chroma scale (decode)
static volatile float g_luma=1.0f;                 // render brightness gain (Luma slider, both modes)
static volatile float g_satf=1.0f;                 // render saturation (Sat slider, both modes)
static uint8_t shared_rgba[YUV_RGBA_BYTES];        // decoded YUV frame for the GPU (filled by engine)
static void yuv_to_rgba(const uint8_t*f,uint8_t*rgba);   // defined after the clamp helper

// ---------- transport (gspca sn9c20x) ----------
static void reg_w(uint16_t reg,const uint8_t*b,int len){ if(g_err)return;
    int r=libusb_control_transfer(dev,LIBUSB_ENDPOINT_OUT|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_RECIPIENT_INTERFACE,0x08,reg,0,(unsigned char*)b,len,500);
    if(r!=len){fprintf(stderr,"reg_w %04x %d\n",reg,r);g_err=r;} }
static void reg_w1(uint16_t reg,uint8_t v){reg_w(reg,&v,1);}
static int reg_r(uint16_t reg,uint8_t*b,int len){ if(g_err)return -1;
    int r=libusb_control_transfer(dev,LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_RECIPIENT_INTERFACE,0x00,reg,0,b,len,500);
    if(r<0)g_err=r; return r; }
static void i2c_w(const uint8_t row[8]){ uint8_t s[1]; reg_w(0x10c0,row,8);
    for(int i=0;i<5;i++){ if(reg_r(0x10c0,s,1)<0)return; if(s[0]&0x04){if(s[0]&0x08)g_err=-1;return;} usleep(10000);} }
static void i2c_w2(uint8_t reg,uint16_t val){ uint8_t row[8]={I2C_INTF|(3<<4),I2C_ADDR,reg,val>>8,val&0xff,0,0,0x10}; i2c_w(row); }
// Bridge YUV color matrix (reg 0x10e1) - REQUIRED for chroma in YUV mode; without it U/V come out
// constant. Ported from sn9c20x set_cmatrix; hsv tables hard-coded at hue index 180 (hue=0).
static void set_cmatrix(int brightness,int contrast,int satur){
    const int rx=-41,ry=-82,gx=124,gy=100,bx=-112,by=11;   // hsv_*[180]
    uint8_t cm[21]; memset(cm,0,sizeof(cm)); int hc;
    cm[2]=(contrast*0x25/0x100)+0x26;
    cm[0]=0x13+(cm[2]-0x26)*0x13/0x25;
    cm[4]=0x07+(cm[2]-0x26)*0x07/0x25;
    cm[18]=(uint8_t)(brightness-0x80);
    hc=(rx*satur)>>8; cm[6]=hc;  cm[7]=(hc>>8)&0x0f;
    hc=(ry*satur)>>8; cm[8]=hc;  cm[9]=(hc>>8)&0x0f;
    hc=(gx*satur)>>8; cm[10]=hc; cm[11]=(hc>>8)&0x0f;
    hc=(gy*satur)>>8; cm[12]=hc; cm[13]=(hc>>8)&0x0f;
    hc=(bx*satur)>>8; cm[14]=hc; cm[15]=(hc>>8)&0x0f;
    hc=(by*satur)>>8; cm[16]=hc; cm[17]=(hc>>8)&0x0f;
    reg_w(0x10e1,cm,21);
}
static void set_gamma(int val){
    uint8_t g[17]; int gv = val * 0xb8 / 0x100;
    g[0] = 0x0a;
    g[1]  = 0x13 + (gv * (0xcb - 0x13) / 0xb8);  g[2]  = 0x25 + (gv * (0xee - 0x25) / 0xb8);
    g[3]  = 0x37 + (gv * (0xfa - 0x37) / 0xb8);  g[4]  = 0x45 + (gv * (0xfc - 0x45) / 0xb8);
    g[5]  = 0x55 + (gv * (0xfb - 0x55) / 0xb8);  g[6]  = 0x65 + (gv * (0xfc - 0x65) / 0xb8);
    g[7]  = 0x74 + (gv * (0xfd - 0x74) / 0xb8);  g[8]  = 0x83 + (gv * (0xfe - 0x83) / 0xb8);
    g[9]  = 0x92 + (gv * (0xfc - 0x92) / 0xb8);  g[10] = 0xa1 + (gv * (0xfc - 0xa1) / 0xb8);
    g[11] = 0xb0 + (gv * (0xfc - 0xb0) / 0xb8);  g[12] = 0xbf + (gv * (0xfb - 0xbf) / 0xb8);
    g[13] = 0xce + (gv * (0xfb - 0xce) / 0xb8);  g[14] = 0xdf + (gv * (0xfd - 0xdf) / 0xb8);
    g[15] = 0xea + (gv * (0xf9 - 0xea) / 0xb8);  g[16] = 0xf5;
    reg_w(0x1190,g,17);
}
static const uint16_t bridge_init[][2]={
    {0x1000,0x78},{0x1001,0x40},{0x1002,0x1c},{0x1020,0x80},{0x1061,0x01},{0x1067,0x40},{0x1068,0x30},{0x1069,0x20},{0x106a,0x10},{0x106b,0x08},
    {0x1188,0x87},{0x11a1,0x00},{0x11a2,0x00},{0x11a3,0x6a},{0x11a4,0x50},{0x11ab,0x00},{0x11ac,0x00},{0x11ad,0x50},{0x11ae,0x3c},{0x118a,0x04},
    {0x0395,0x04},{0x11b8,0x3a},{0x118b,0x0e},{0x10f7,0x05},{0x10f8,0x14},{0x10fa,0xff},{0x10f9,0x00},{0x11ba,0x0a},{0x11a5,0x2d},{0x11a6,0x2d},
    {0x11a7,0x3a},{0x11a8,0x05},{0x11a9,0x04},{0x11aa,0x3f},{0x11af,0x28},{0x11b0,0xd8},{0x11b1,0x14},{0x11b2,0xec},{0x11b3,0x32},{0x11b4,0xdd},
    {0x11b5,0x32},{0x11b6,0xdd},{0x10e0,0x2c},{0x11bc,0x40},{0x11bd,0x01},{0x11be,0xf0},{0x11bf,0x00},{0x118c,0x1f},{0x118d,0x1f},{0x118e,0x1f},
    {0x118f,0x1f},{0x1180,0x01},{0x1181,0x00},{0x1182,0x01},{0x1183,0x00},{0x1184,0x50},{0x1185,0x80},{0x1007,0x00} };
static const uint16_t mt9m111_init[][2]={
    {0xf0,0x0000},{0x0d,0x0021},{0x0d,0x0008},{0xf0,0x0001},{0x3a,0x4300},{0x9b,0x4300},{0x06,0x708e},{0xf0,0x0002},{0x2e,0x0a1e},{0xf0,0x0000} };
static void dino_init(void){ uint8_t v;
    for(unsigned i=0;i<sizeof(bridge_init)/sizeof(bridge_init[0]);i++){v=bridge_init[i][1];reg_w(bridge_init[i][0],&v,1);}
    reg_w1(0x1006,g_led); g_led_applied=g_led;
    uint8_t ii[9]={0x80,I2C_ADDR,0,0,0,0,0,0,0x03}; reg_w(0x10c0,ii,9);
    for(unsigned i=0;i<sizeof(mt9m111_init)/sizeof(mt9m111_init[0]);i++) i2c_w2(mt9m111_init[i][0],mt9m111_init[i][1]); }
static void dino_start(void){ int hs=0,vs=2;
    i2c_w2(0xf0,0x0002); i2c_w2(0xc8,0x970b); i2c_w2(0xf0,0x0000);
    uint8_t clr[5]={0,(W>>2)&0xff,0,(H>>1)&0xff,(uint8_t)(((W>>10)&1)|((H>>8)&6))}; reg_w(0x10fb,clr,5);
    uint8_t hw[6]={(uint8_t)hs,0,(uint8_t)vs,0,(W>>4)&0xff,(H>>3)&0xff}; reg_w(0x1180,hw,6);
    reg_w1(0x1189,0xc0); reg_w1(0x10e0,0x2d);   // scale 1280x1024, fmt 0x2d = RAW Bayer
    // sn9c20x sd_start writes the bridge color regs on EVERY stream start, all modes incl RAW
    // (sn9c20x.c:2046-2052). They reset on a replug power-cycle, so re-lay them here or cold SXGA
    // shows default (cast) color - that was the replug-color bug.
    set_cmatrix(127,127,g_sat); set_gamma(0x10);                    // 0x10e1 cmatrix / 0x1190 gamma
    reg_w1(0x118c,(uint8_t)g_red); reg_w1(0x118f,(uint8_t)g_blue);  // set_redblue WB gains
    i2c_w2(0xf0,0x0000); i2c_w2(0x09,(uint16_t)g_exp);              // one-shot exposure baseline; software AE drives 0x09 after
    reg_w1(0x1007,0x20); reg_w1(0x1061,0x03); }
// YUV 420 hardware-demosaic mode @ 640x480 (sn9c20x sd_start, non-SXGA, fmt 0x2f / scale 0x80).
static void dino_start_yuv(void){
    i2c_w2(0xf0,0x0002); i2c_w2(0xc8,0x8000); i2c_w2(0xf0,0x0000);   // MT9M111 non-SXGA output context
    uint8_t clr[5]={0,(YUV_W>>2)&0xff,0,(YUV_WIN_H>>1)&0xff,(uint8_t)(((YUV_W>>10)&1)|((YUV_WIN_H>>8)&6))}; reg_w(0x10fb,clr,5);
    uint8_t hw[6]={0,0,0,0,(YUV_W>>4)&0xff,(YUV_WIN_H>>3)&0xff}; reg_w(0x1180,hw,6);
    reg_w1(0x1189,0x80);    // scale = 640x480
    reg_w1(0x10e0,0x2f);    // fmt = YUV 420
    set_cmatrix(127,127,g_sat);   // bridge color matrix (enables chroma)
    set_gamma(0x10);
    reg_w1(0x118c,(uint8_t)g_red); reg_w1(0x118f,(uint8_t)g_blue);   // set_redblue: WB gains
    i2c_w2(0xf0,0x0000); i2c_w2(0x09,(uint16_t)g_exp); i2c_w2(0xf0,0x0000);   // exposure baseline
    reg_w1(0x1007,0x20); reg_w1(0x1061,0x03); }
static void dino_stop(void){ reg_w1(0x1007,0x00); reg_w1(0x1061,0x01); }

// ---------- frame assembler ----------
static const uint8_t FHDR[6]={0xff,0xff,0x00,0xc4,0xc4,0x96};   // same Sonix frame header in both modes
static uint8_t framebuf[FRAME_BYTES+W*4]; static int fill=0;
static uint8_t g_rgba_dec[YUV_RGBA_BYTES];   // engine-thread YUV->RGBA scratch
static void emit(void){
    int target = g_mode ? YUV_FRAME_BYTES : FRAME_BYTES;
    if(fill>=target){
        if(g_mode){ yuv_to_rgba(framebuf,g_rgba_dec);
            pthread_mutex_lock(&lock); memcpy(shared_rgba,g_rgba_dec,YUV_RGBA_BYTES); shared_new=1; pthread_mutex_unlock(&lock); }
        else { pthread_mutex_lock(&lock); memcpy(shared_raw,framebuf,FRAME_BYTES); shared_new=1; pthread_mutex_unlock(&lock); }
        g_frames++;
    }
    fill=0; }
static void feed(uint8_t*d,int len){ if(len>=64&&memcmp(d,FHDR,6)==0){emit();d+=64;len-=64;} if(len<=0)return;
    if(fill+len>(int)sizeof(framebuf))len=sizeof(framebuf)-fill; memcpy(framebuf+fill,d,len); fill+=len; }
static volatile int g_streaming=0;
static struct libusb_transfer*g_tr[16];   // iso transfer slots (engine thread only); iso_cb NULLs its own
// A transfer is "reaped" when its callback fires and we do NOT resubmit. We free it HERE (not in
// stop_stream) so libusb never holds a freed/in-flight transfer at close time - that was the unplug
// crash (libusb_close asserted on still-flying transfers). g_inflight tracks how many libusb owns.
static void LIBUSB_CALL iso_cb(struct libusb_transfer*t){
    if(g_streaming && t->status!=LIBUSB_TRANSFER_NO_DEVICE){
        for(int i=0;i<t->num_iso_packets;i++){ struct libusb_iso_packet_descriptor*p=&t->iso_packet_desc[i];
            if(p->status==LIBUSB_TRANSFER_COMPLETED&&p->actual_length>0) feed(libusb_get_iso_packet_buffer_simple(t,i),p->actual_length); }
        if(libusb_submit_transfer(t)==0) return;   // resubmitted: still in flight
        g_devlost=1;   // resubmit failed -> device gone/bad, reconnect
    }
    if(t->status==LIBUSB_TRANSFER_NO_DEVICE) g_devlost=1;   // unplugged
    int idx=(int)(intptr_t)t->user_data;   // NULL our slot so stop_stream won't cancel a freed transfer
    if(idx>=0&&idx<16) g_tr[idx]=NULL;
    free(t->buffer); libusb_free_transfer(t); g_inflight--; }   // reap: stop owns it now
// iso EP 0x81 packet size (with high-bandwidth multiplier) for an altsetting
static int ep_pktsize(int alt){
    libusb_device*d=libusb_get_device(dev); struct libusb_config_descriptor*cfg;
    if(libusb_get_active_config_descriptor(d,&cfg))return -1; int sz=-1;
    for(int i=0;i<cfg->bNumInterfaces;i++){ const struct libusb_interface*itf=&cfg->interface[i];
        for(int a=0;a<itf->num_altsetting;a++){ const struct libusb_interface_descriptor*id=&itf->altsetting[a];
            if(id->bInterfaceNumber!=IFACE||id->bAlternateSetting!=alt)continue;
            for(int e=0;e<id->bNumEndpoints;e++){ const struct libusb_endpoint_descriptor*ep=&id->endpoint[e];
                if(ep->bEndpointAddress==EP_ISO){ int mps=ep->wMaxPacketSize&0x7ff; int mult=((ep->wMaxPacketSize>>11)&3)+1; sz=mps*mult; } } } }
    libusb_free_config_descriptor(cfg); return sz;
}
static int g_nt=0;
static void start_stream(void){
    int alt,nt,np,psz;
    if(g_mode){ dino_start_yuv(); alt=9; psz=ep_pktsize(alt); if(psz<=0){alt=8; psz=ep_pktsize(alt);} if(psz<=0)psz=5120; nt=8; np=64; }
    // Both start paths lay down the bridge color baseline (cmatrix/gamma/redblue/exposure) like the
    // kernel's sd_start, so a replug power-cycle is covered for SXGA and YUV alike.
    else { dino_start(); alt=ALT; psz=5120; nt=10; np=64; }
    libusb_set_interface_alt_setting(dev,IFACE,alt);
    fill=0; g_streaming=1; g_nt=nt; g_inflight=0;
    for(int i=0;i<nt;i++){ g_tr[i]=libusb_alloc_transfer(np); uint8_t*b=malloc((size_t)np*psz);
        libusb_fill_iso_transfer(g_tr[i],dev,EP_ISO,b,np*psz,np,iso_cb,(void*)(intptr_t)i,0); libusb_set_iso_packet_lengths(g_tr[i],psz);
        if(libusb_submit_transfer(g_tr[i])==0) g_inflight++; else { free(b); libusb_free_transfer(g_tr[i]); g_tr[i]=NULL; } }
    g_mode_applied=g_mode; g_exp_applied=-1; }
static void stop_stream(void){
    g_streaming=0;
    for(int i=0;i<g_nt;i++) if(g_tr[i]) libusb_cancel_transfer(g_tr[i]);
    // reap: pump events until every transfer's callback has freed it (iso_cb frees, decs g_inflight).
    // Do NOT free here - freeing an in-flight transfer is what made libusb_close assert on unplug.
    for(int t=0; g_inflight>0 && t<200; t++){ struct timeval tv={0,10000}; libusb_handle_events_timeout(ctx,&tv); }
    g_nt=0;
    if(!g_devlost){ libusb_set_interface_alt_setting(dev,IFACE,0); dino_stop(); }   // skip control I/O if unplugged
    fill=0; }
// post a status line to the UI label from the engine thread (AppKit is main-thread only)
static NSTextField *g_status_label=nil;
static void set_status(NSString*s){ dispatch_async(dispatch_get_main_queue(), ^{ if(g_status_label) g_status_label.stringValue=s; }); }
// open + claim the device; reset as a fallback if the interface is stuck (half-claim after a replug)
static int open_device(void){
    dev=libusb_open_device_with_vid_pid(ctx,VID,PID);
    if(!dev) return -1;
    libusb_set_auto_detach_kernel_driver(dev,1);
    if(libusb_claim_interface(dev,IFACE)){
        libusb_reset_device(dev);
        if(libusb_claim_interface(dev,IFACE)){ libusb_close(dev); dev=NULL; return -1; }
    }
    return 0;
}
static void* engine(void*a){
    if(libusb_init(&ctx)){fprintf(stderr,"libusb_init fail\n");return 0;}
    // The physical touch button reports only on INT EP 0x83, which macOS libusb never delivers,
    // so capture is triggered from the UI (Capture button / Spacebar) instead.
    while(g_run){
        // (re)connect: wait for the device to appear (covers first launch and replug)
        if(open_device()<0){ set_status(@"Waiting for device — plug in the Dino-Lite…"); usleep(400000); continue; }
        g_err=0; g_devlost=0; g_led_applied=-1; g_mode_applied=-1; g_exp_applied=-1;
        dino_init();
        if(g_err){ libusb_close(dev); dev=NULL; usleep(400000); continue; }   // flaky enumeration: retry
        start_stream();
        set_status(g_mode?@"Connected — YUV 640×472":@"Connected — SXGA 1280×1023");
        unsigned lastf=g_frames; int stall=0;
        while(g_run && !g_devlost){
            struct timeval tv={0,20000}; libusb_handle_events_timeout(ctx,&tv);
            if(g_frames!=lastf){ lastf=g_frames; stall=0; } else if(++stall>150) g_devlost=1;   // ~3s of no frames = lost
            if(g_led!=g_led_applied){ reg_w1(0x1006,g_led); g_led_applied=g_led; }
            if(g_mode!=g_mode_applied){ stop_stream(); start_stream(); }   // live mode switch
            if(g_mode && g_exp!=g_exp_applied){ i2c_w2(0xf0,0x0000); i2c_w2(0x09,(uint16_t)g_exp); g_exp_applied=g_exp; }
        }
        // teardown this connection (graceful on quit, best-effort on unplug)
        if(g_run) set_status(@"Device lost — reconnecting…");
        stop_stream();
        if(!g_devlost) reg_w1(0x1006,0x00);   // turn LED off on a clean stop only
        if(dev){ if(!g_devlost) libusb_release_interface(dev,IFACE); libusb_close(dev); dev=NULL; }   // skip release if unplugged
    }
    libusb_exit(ctx); return 0; }

// ---------- white-patch AWB (CPU, on the raw Bayer) ----------
static void compute_awb(const uint8_t*raw){
    // white-patch on bright-but-unclipped pixels; heavy smoothing => locks, no flicker
    long long sR=0,sG=0,sB=0,n=0; int mx=1;
    static int thr=150;
    for(int y=0;y+1<CH;y+=2) for(int x=0;x+1<W;x+=2){
        int p00=raw[y*W+x],p10=raw[y*W+x+1],p01=raw[(y+1)*W+x],p11=raw[(y+1)*W+x+1], R,G,B;
        switch(g_phase){ case 0: G=(p00+p11)>>1; B=p10; R=p01; break;
                         case 1: G=(p00+p11)>>1; R=p10; B=p01; break;
                         case 2: R=p00; G=(p10+p01)>>1; B=p11; break;
                         default: B=p00; G=(p10+p01)>>1; R=p11; break; }
        int lum=(R+2*G+B)>>2; if(lum>mx)mx=lum;
        if(lum>=thr && R<248 && G<248 && B<248){ sR+=R; sG+=G; sB+=B; n++; }  // skip clipped
    }
    thr=mx*45/100; if(thr<20)thr=20;
    if(n<50) return;
    double mR=(double)sR/n+1e-6, mG=(double)sG/n+1e-6, mB=(double)sB/n+1e-6;
    double ref=mR; if(mG>ref)ref=mG; if(mB>ref)ref=mB;
    float tR=ref/mR, tG=ref/mG, tB=ref/mB;
    if(tR>4)tR=4; if(tG>4)tG=4; if(tB>4)tB=4;
    if(fabsf(tR-g_gR)<0.02f && fabsf(tG-g_gG)<0.02f && fabsf(tB-g_gB)<0.02f) return;
    g_gR=g_gR*0.8f+tR*0.2f; g_gG=g_gG*0.8f+tG*0.2f; g_gB=g_gB*0.8f+tB*0.2f;
}

// ---------- CPU debayer for capture ----------
static inline int cpx(const uint8_t*b,int x,int y){ if(x<0)x=0;if(x>=W)x=W-1;if(y<0)y=0;if(y>=CH)y=CH-1;return b[y*W+x]; }
static inline uint8_t clp(int v){return v<0?0:(v>255?255:v);}

// ---------- SN9C20X custom YUV420 -> RGBA (BT.601), used by YUV mode ----------
// 16-wide x 8-tall macroblocks in raster order, 192 bytes each: 128 Y (two 8x8 halves, left then
// right) + 32 U + 32 V (8-wide x 4-tall chroma block). buf[i+0..127]=Y, [i+128..159]=U, [i+160..191]=V.
static uint8_t g_Yp[YUV_W*YUV_H], g_Cbp[(YUV_W/2)*(YUV_H/2)], g_Crp[(YUV_W/2)*(YUV_H/2)];
static void yuv_to_rgba(const uint8_t*f,uint8_t*rgba){
    int cw=YUV_W/2, i=0,x=0,y=0;
    while(i+192<=YUV_FRAME_BYTES){
        for(int j=0;j<128;j++){ int dx,dy; if(j<64){dx=j&7;dy=j>>3;} else {int k=j-64;dx=8+(k&7);dy=k>>3;}
            int px=x+dx,py=y+dy; if(px<YUV_W&&py<YUV_H) g_Yp[py*YUV_W+px]=f[i+j]; }
        for(int j=0;j<32;j++){ int cx=(x>>1)+(j&7), cy=(y>>1)+(j>>3);
            if(cx<cw&&cy<YUV_H/2){ g_Cbp[cy*cw+cx]=f[i+128+j]; g_Crp[cy*cw+cx]=f[i+160+j]; } }
        i+=192; x+=16; if(x>=YUV_W){x=0;y+=8;}
    }
    // software white-balance: bridge chroma neutral is offset from 128 and maps to (U,V) as
    // U=-(Cr-neutral), V=(Cb-neutral). Find the neutral from the brightest (white) cells, subtract, scale up.
    int chh=YUV_H/2; double scb=0,scr=0; long cn=0;
    for(int cy=0;cy<chh;cy++)for(int cx=0;cx<cw;cx++)
        if(g_Yp[(cy*2)*YUV_W+(cx*2)]>=200){ scb+=g_Cbp[cy*cw+cx]; scr+=g_Crp[cy*cw+cx]; cn++; }
    float cbn=cn>50?(float)(scb/cn):128.f, crn=cn>50?(float)(scr/cn):128.f, s=g_cscale;
    for(int yy=0;yy<YUV_H;yy++)for(int xx=0;xx<YUV_W;xx++){
        int Y=g_Yp[yy*YUV_W+xx];
        float U=-((float)g_Crp[(yy>>1)*cw+(xx>>1)]-crn)*s;
        float V= ((float)g_Cbp[(yy>>1)*cw+(xx>>1)]-cbn)*s;
        uint8_t*p=rgba+(yy*YUV_W+xx)*4;
        p[0]=clp((int)(Y+1.402f*V)); p[1]=clp((int)(Y-0.344f*U-0.714f*V)); p[2]=clp((int)(Y+1.772f*U)); p[3]=255;
    }
}
static inline float srgbf(float v){ v=v<0?0:(v>1?1:v); return v<=0.0031308f?12.92f*v:1.055f*powf(v,1.0f/2.4f)-0.055f; }
static void cpu_debayer(const uint8_t*raw,uint8_t*rgba,float gR,float gG,float gB,int gray,int phase,float contrast){
    for(int y=0;y<CH;y++)for(int x=0;x<W;x++){
        float R,G,B;
        if(gray){ R=G=B=raw[y*W+x]; }
        else{
            int bx=x&~1; if(bx>W-2)bx=W-2; int by=y&~1; if(by>CH-2)by=CH-2;
            float p00=cpx(raw,bx,by),p10=cpx(raw,bx+1,by),p01=cpx(raw,bx,by+1),p11=cpx(raw,bx+1,by+1);
            float r,g,b;  // phase 0=GBRG 1=GRBG 2=RGGB 3=BGGR
            if(phase==0){ g=(p00+p11)*0.5f; b=p10; r=p01; }
            else if(phase==1){ g=(p00+p11)*0.5f; r=p10; b=p01; }
            else if(phase==2){ r=p00; g=(p10+p01)*0.5f; b=p11; }
            else { b=p00; g=(p10+p01)*0.5f; r=p11; }
            float bl=0.299f*r+0.587f*g+0.114f*b;
            float lp=(cpx(raw,x,y)+cpx(raw,x-1,y)+cpx(raw,x+1,y)+cpx(raw,x,y-1)+cpx(raw,x,y+1))*0.2f;
            float k=(bl>1.0f)?(lp/bl):1.0f; if(k<0.4f)k=0.4f; if(k>2.0f)k=2.0f;
            R=r*k*gR; G=g*k*gG; B=b*k*gB;
        }
        float cr=(srgbf(R/255.0f)-0.5f)*contrast+0.5f;
        float cg=(srgbf(G/255.0f)-0.5f)*contrast+0.5f;
        float cb=(srgbf(B/255.0f)-0.5f)*contrast+0.5f;
        uint8_t*p=rgba+(y*W+x)*4; p[0]=clp((int)(cr*255.0f));p[1]=clp((int)(cg*255.0f));p[2]=clp((int)(cb*255.0f));p[3]=255;
    }
}

// ---------- Metal ----------
typedef struct { int iw,ih,gray,phase; float gR,gG,gB,contrast,luma,satf; } Uniforms;
typedef struct { float luma,satf; } BlitU;   // YUV blit fragment uniforms
typedef struct { float x,y; } V2;             // vertex letterbox scale (matches Metal float2)
static NSString *kShader = @
"#include <metal_stdlib>\n"
"using namespace metal;\n"
"struct Uniforms { int iw; int ih; int gray; int phase; float gR; float gG; float gB; float contrast; float luma; float satf; };\n"
"struct BlitU { float luma; float satf; };\n"
"struct VOut { float4 pos [[position]]; float2 uv; };\n"
// scale = letterbox fit factors; geometry stays full-screen, uv is divided by scale so the image
// occupies a centered rect of the view's aspect and the bars fall outside [0,1] (drawn black below).
"vertex VOut vmain(uint vid [[vertex_id]], constant float2& scale [[buffer(0)]]){\n"
"  float2 p[3]={float2(-1,-1),float2(3,-1),float2(-1,3)};\n"
"  VOut o; o.pos=float4(p[vid],0,1); float2 q=p[vid]/scale; float2 t=q*0.5+0.5; o.uv=float2(t.x,1.0-t.y); return o; }\n"
"static inline float3 grade(float3 c,float satf,float luma){\n"   // saturation about luma, then brightness
"  float Y=dot(c,float3(0.299,0.587,0.114)); c=clamp(Y+(c-Y)*satf,0.0,1.0); return clamp(c*luma,0.0,1.0); }\n"
"static inline float rd(texture2d<float,access::read> b,int x,int y,int W,int H){\n"
"  x=clamp(x,0,W-1); y=clamp(y,0,H-1); return b.read(uint2(x,y)).r*255.0; }\n"
"fragment float4 fmain(VOut in [[stage_in]], texture2d<float,access::read> bayer [[texture(0)]], constant Uniforms& u [[buffer(0)]]){\n"
"  if(in.uv.x<0.0||in.uv.x>1.0||in.uv.y<0.0||in.uv.y>1.0) return float4(0,0,0,1);\n"   // letterbox bar
"  int X=clamp(int(in.uv.x*float(u.iw)),0,u.iw-1); int Y=clamp(int(in.uv.y*float(u.ih)),0,u.ih-1);\n"
"  float R,G,B;\n"
"  if(u.gray!=0){ R=G=B=rd(bayer,X,Y,u.iw,u.ih); }\n"
"  else {\n"
"    int bx=min(X&~1,u.iw-2); int by=min(Y&~1,u.ih-2);\n"
"    float p00=rd(bayer,bx,by,u.iw,u.ih), p10=rd(bayer,bx+1,by,u.iw,u.ih), p01=rd(bayer,bx,by+1,u.iw,u.ih), p11=rd(bayer,bx+1,by+1,u.iw,u.ih);\n"
"    float r,g,b;\n"   // 0=GBRG 1=GRBG 2=RGGB 3=BGGR
"    if(u.phase==0){ g=(p00+p11)*0.5; b=p10; r=p01; }\n"
"    else if(u.phase==1){ g=(p00+p11)*0.5; r=p10; b=p01; }\n"
"    else if(u.phase==2){ r=p00; g=(p10+p01)*0.5; b=p11; }\n"
"    else { b=p00; g=(p10+p01)*0.5; r=p11; }\n"
// luma detail at full pixel res, chroma from block -> sharp luma, clean color
"    float bl=0.299*r+0.587*g+0.114*b;\n"
"    float lp=(rd(bayer,X,Y,u.iw,u.ih)+rd(bayer,X-1,Y,u.iw,u.ih)+rd(bayer,X+1,Y,u.iw,u.ih)+rd(bayer,X,Y-1,u.iw,u.ih)+rd(bayer,X,Y+1,u.iw,u.ih))*0.2;\n"
"    float k=(bl>1.0)?clamp(lp/bl,0.4,2.0):1.0;\n"
"    R=r*k*u.gR; G=g*k*u.gG; B=b*k*u.gB;\n"
"  }\n"
// linear sensor RAW -> sRGB transfer (gamma) + contrast about mid-gray
"  float3 c=clamp(float3(R,G,B)/255.0,0.0,1.0);\n"
"  c=select(1.055*pow(c,1.0/2.4)-0.055, 12.92*c, c<=0.0031308);\n"
"  c=clamp((c-0.5)*u.contrast+0.5,0.0,1.0);\n"
"  c=grade(c,u.satf,u.luma);\n"
"  return float4(c,1.0); }\n"
// YUV mode: the frame is already debayered/color-corrected on CPU into an RGBA texture; just sample it.
"fragment float4 fblit(VOut in [[stage_in]], texture2d<float> rgb [[texture(0)]], constant BlitU& u [[buffer(0)]]){\n"
"  if(in.uv.x<0.0||in.uv.x>1.0||in.uv.y<0.0||in.uv.y>1.0) return float4(0,0,0,1);\n"   // letterbox bar
"  constexpr sampler s(coord::normalized, filter::linear, address::clamp_to_edge);\n"
"  float3 c=rgb.sample(s,in.uv).rgb; c=grade(c,u.satf,u.luma);\n"
"  return float4(c,1.0); }\n";

@interface Renderer : NSObject <MTKViewDelegate>
@property(strong) id<MTLDevice> dev;
@property(strong) id<MTLCommandQueue> q;
@property(strong) id<MTLRenderPipelineState> pso;     // SXGA Bayer debayer pipeline
@property(strong) id<MTLRenderPipelineState> psoBlit; // YUV blit pipeline
@property(strong) id<MTLTexture> bayerTex;
@property(strong) id<MTLTexture> yuvTex;              // RGBA8 640x472 for YUV mode
@property(strong) NSTextField *status;
@property(assign) uint8_t *upbuf;     // staging copy of latest raw Bayer
@property(assign) uint8_t *upRGBA;    // staging copy of latest decoded YUV
@end
@implementation Renderer
-(instancetype)initWithView:(MTKView*)v{
    if(!(self=[super init]))return nil;
    self.dev=v.device; self.q=[self.dev newCommandQueue];
    NSError*e=nil; id<MTLLibrary> lib=[self.dev newLibraryWithSource:kShader options:nil error:&e];
    if(!lib){ NSLog(@"shader err: %@",e); return nil; }
    MTLRenderPipelineDescriptor*d=[MTLRenderPipelineDescriptor new];
    d.vertexFunction=[lib newFunctionWithName:@"vmain"]; d.fragmentFunction=[lib newFunctionWithName:@"fmain"];
    d.colorAttachments[0].pixelFormat=v.colorPixelFormat;
    self.pso=[self.dev newRenderPipelineStateWithDescriptor:d error:&e];
    if(!self.pso){ NSLog(@"pso err: %@",e); return nil; }
    MTLRenderPipelineDescriptor*db=[MTLRenderPipelineDescriptor new];
    db.vertexFunction=[lib newFunctionWithName:@"vmain"]; db.fragmentFunction=[lib newFunctionWithName:@"fblit"];
    db.colorAttachments[0].pixelFormat=v.colorPixelFormat;
    self.psoBlit=[self.dev newRenderPipelineStateWithDescriptor:db error:&e];
    if(!self.psoBlit){ NSLog(@"psoBlit err: %@",e); return nil; }
    MTLTextureDescriptor*td=[MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:W height:CH mipmapped:NO];
    td.usage=MTLTextureUsageShaderRead;
    self.bayerTex=[self.dev newTextureWithDescriptor:td];
    MTLTextureDescriptor*ty=[MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:YUV_W height:YUV_H mipmapped:NO];
    ty.usage=MTLTextureUsageShaderRead;
    self.yuvTex=[self.dev newTextureWithDescriptor:ty];
    self.upbuf=malloc(FRAME_BYTES); self.upRGBA=malloc(YUV_RGBA_BYTES);
    return self;
}
-(void)mtkView:(MTKView*)v drawableSizeWillChange:(CGSize)s{}
-(void)drawInMTKView:(MTKView*)view{
    int mode=g_mode, have=0;
    pthread_mutex_lock(&lock);
    if(shared_new){ if(mode) memcpy(self.upRGBA,shared_rgba,YUV_RGBA_BYTES); else memcpy(self.upbuf,shared_raw,FRAME_BYTES); shared_new=0; have=1; }
    pthread_mutex_unlock(&lock);
    if(have){ if(mode) [self.yuvTex replaceRegion:MTLRegionMake2D(0,0,YUV_W,YUV_H) mipmapLevel:0 withBytes:self.upRGBA bytesPerRow:YUV_W*4];
              else      [self.bayerTex replaceRegion:MTLRegionMake2D(0,0,W,CH) mipmapLevel:0 withBytes:self.upbuf bytesPerRow:W]; }
    // letterbox: fit the image aspect inside the view aspect (centered, black bars), no stretch
    float iw=mode?YUV_W:W, ih=mode?YUV_H:CH; CGSize ds=view.drawableSize;
    float Av=(ds.height>0)?(float)(ds.width/ds.height):1.0f, Ai=iw/ih; V2 scale;
    if(Av>Ai){ scale.x=Ai/Av; scale.y=1.0f; } else { scale.x=1.0f; scale.y=Av/Ai; }
    id<MTLCommandBuffer> cb=[self.q commandBuffer];
    MTLRenderPassDescriptor*rp=view.currentRenderPassDescriptor;
    if(rp){
        id<MTLRenderCommandEncoder> en=[cb renderCommandEncoderWithDescriptor:rp];
        [en setVertexBytes:&scale length:sizeof(scale) atIndex:0];
        if(mode){
            [en setRenderPipelineState:self.psoBlit];
            [en setFragmentTexture:self.yuvTex atIndex:0];
            BlitU bu={g_luma,g_satf}; [en setFragmentBytes:&bu length:sizeof(bu) atIndex:0];
        } else {
            [en setRenderPipelineState:self.pso];
            Uniforms u={W,CH,g_gray,g_phase,g_gR,g_gG,g_gB,g_contrast,g_luma,g_satf};
            [en setFragmentTexture:self.bayerTex atIndex:0];
            [en setFragmentBytes:&u length:sizeof(u) atIndex:0];
        }
        [en drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [en endEncoding];
        [cb presentDrawable:view.currentDrawable];
    }
    [cb commit];
    if(g_capture_req){ g_capture_req=0; [self saveShot]; }
}
-(BOOL)savePNG:(NSString*)path{
    int mode=g_mode, w=mode?YUV_W:W, h=mode?YUV_H:CH;
    static uint8_t rgba[W*CH*4];
    pthread_mutex_lock(&lock);
    if(mode) memcpy(rgba,shared_rgba,YUV_RGBA_BYTES);
    else     cpu_debayer(shared_raw,rgba,g_gR,g_gG,g_gB,g_gray,g_phase,g_contrast);
    pthread_mutex_unlock(&lock);
    BOOL ok=NO; CGColorSpaceRef cs=CGColorSpaceCreateDeviceRGB();
    CGContextRef c=CGBitmapContextCreate(NULL,w,h,8,w*4,cs,kCGImageAlphaNoneSkipLast|kCGBitmapByteOrderDefault);
    if(c){ memcpy(CGBitmapContextGetData(c),rgba,(size_t)w*h*4); CGImageRef im=CGBitmapContextCreateImage(c);
        if(im){ CFURLRef url=CFURLCreateWithFileSystemPath(NULL,(__bridge CFStringRef)path,kCFURLPOSIXPathStyle,false);
            CGImageDestinationRef d=CGImageDestinationCreateWithURL(url,CFSTR("public.png"),1,NULL);
            if(d){ CGImageDestinationAddImage(d,im,NULL); ok=CGImageDestinationFinalize(d); CFRelease(d);} if(url)CFRelease(url); CGImageRelease(im);} CGContextRelease(c);}
    CGColorSpaceRelease(cs); return ok;
}
-(void)saveShot{
    NSDateFormatter*df=[NSDateFormatter new]; df.dateFormat=@"yyyyMMdd_HHmmss";
    NSString*path=[NSString stringWithFormat:@"%@/Desktop/Dino_%@.png",NSHomeDirectory(),[df stringFromDate:[NSDate date]]];
    BOOL ok=[self savePNG:path];
    self.status.stringValue=ok?[NSString stringWithFormat:@"Saved %@",path.lastPathComponent]:@"Save FAILED";
}
-(void)awbTick{ static uint8_t cp[FRAME_BYTES]; pthread_mutex_lock(&lock); memcpy(cp,shared_raw,FRAME_BYTES); pthread_mutex_unlock(&lock); compute_awb(cp); }
@end

@interface Btns:NSObject @property(strong) Renderer* r; @end
@implementation Btns
-(void)cap:(id)s{ g_capture_req=1; }
-(void)led:(NSSegmentedControl*)s{ g_led=s.selectedSegment?0x20:0x00; }  // AM411T LED: 0x20=on 0x00=off (GPIO, no brightness)
-(void)gray:(NSButton*)s{ g_gray=(s.state==NSControlStateValueOn); }
-(void)awb:(id)s{ if(g_mode){ self.r.status.stringValue=@"Lock WB: SXGA only (YUV self-balances)"; return; }
    g_awb_n=14; self.r.status.stringValue=@"Auto WB…"; }   // re-converge then hold (SXGA only)
-(void)awbAuto:(id)s{ if(g_awb_n>0){ g_awb_n--; [self.r awbTick];
    if(g_awb_n==0) self.r.status.stringValue=[NSString stringWithFormat:@"WB locked R%.2f G%.2f B%.2f",g_gR,g_gG,g_gB]; } }
-(void)phase:(NSButton*)s{ g_phase=(g_phase+1)&3; const char*nm[4]={"GBRG","GRBG","RGGB","BGGR"};
    s.title=[NSString stringWithFormat:@"Bayer: %s",nm[g_phase]]; g_awb_n=14; }   // re-converge WB for new phase
-(void)contrast:(NSSlider*)s{ g_contrast=s.floatValue; }
-(void)mode:(NSSegmentedControl*)s{ g_mode=(int)s.selectedSegment;   // 0=SXGA Bayer  1=YUV 640
    self.r.status.stringValue=g_mode?@"YUV 640×472 (hw demosaic)":@"SXGA 1280×1023 (sw debayer)"; }
-(void)luma:(NSSlider*)s{ g_luma=s.floatValue; }       // render brightness gain (both modes)
-(void)sat:(NSSlider*)s{ g_satf=s.floatValue; }        // render saturation (both modes)
@end

@interface AppDelegate:NSObject <NSApplicationDelegate> @end
@implementation AppDelegate
-(BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)a{ return YES; }
-(void)applicationWillTerminate:(NSNotification*)n{ g_run=0; pthread_join(g_engine,NULL); }  // engine cleanup turns LED off
@end

int main(int argc,char**argv){
  @autoreleasepool{
    pthread_create(&g_engine,NULL,engine,NULL);
    NSApplication*app=[NSApplication sharedApplication]; [app setActivationPolicy:NSApplicationActivationPolicyRegular];
    AppDelegate*del=[AppDelegate new]; app.delegate=del;
    // Dock icon: load icon.png from the executable's directory (gitignored - local only).
    NSString*exe=(argc>0)?[NSString stringWithUTF8String:argv[0]]:@"";
    NSString*ip=[[exe stringByDeletingLastPathComponent] stringByAppendingPathComponent:@"icon.png"];
    NSImage*icon=[[NSImage alloc] initWithContentsOfFile:ip];
    if(icon) [app setApplicationIconImage:icon];
    NSRect fr=NSMakeRect(0,0,1400,860);
    NSWindow*win=[[NSWindow alloc] initWithContentRect:fr styleMask:NSWindowStyleMaskTitled|NSWindowStyleMaskClosable|NSWindowStyleMaskResizable backing:NSBackingStoreBuffered defer:NO];
    [win setTitle:@"Dino-Lite (Metal)"]; [win center];
    NSView*cv=win.contentView;
    MTKView*mv=[[MTKView alloc] initWithFrame:NSMakeRect(10,70,1380,780) device:MTLCreateSystemDefaultDevice()];
    mv.colorPixelFormat=MTLPixelFormatBGRA8Unorm; mv.preferredFramesPerSecond=30;
    mv.clearColor=MTLClearColorMake(0,0,0,1);   // black letterbox bars
    mv.autoresizingMask=NSViewWidthSizable|NSViewHeightSizable; [cv addSubview:mv];
    Renderer*r=[[Renderer alloc] initWithView:mv]; mv.delegate=r;
    Btns*b=[Btns new]; b.r=r;

    NSButton*cap=[[NSButton alloc] initWithFrame:NSMakeRect(10,20,130,30)]; cap.title=@"Capture (PNG)"; cap.bezelStyle=NSBezelStyleRounded; cap.target=b; cap.action=@selector(cap:); [cv addSubview:cap];
    NSTextField*ml=[NSTextField labelWithString:@"Mode:"]; ml.frame=NSMakeRect(148,25,42,20); [cv addSubview:ml];
    NSSegmentedControl*mseg=[[NSSegmentedControl alloc] initWithFrame:NSMakeRect(190,18,112,30)]; mseg.segmentCount=2;
    [mseg setLabel:@"SXGA" forSegment:0];[mseg setLabel:@"YUV" forSegment:1];
    mseg.selectedSegment=0; mseg.target=b; mseg.action=@selector(mode:); [cv addSubview:mseg];
    NSTextField*ll=[NSTextField labelWithString:@"LED:"]; ll.frame=NSMakeRect(314,25,34,20); [cv addSubview:ll];
    NSSegmentedControl*seg=[[NSSegmentedControl alloc] initWithFrame:NSMakeRect(348,18,96,30)]; seg.segmentCount=2;
    [seg setLabel:@"Off" forSegment:0];[seg setLabel:@"On" forSegment:1];
    seg.selectedSegment=1; seg.target=b; seg.action=@selector(led:); [cv addSubview:seg];
    NSButton*gchk=[NSButton checkboxWithTitle:@"Gray" target:b action:@selector(gray:)]; gchk.frame=NSMakeRect(454,22,58,22); [cv addSubview:gchk];
    NSButton*awb=[[NSButton alloc] initWithFrame:NSMakeRect(516,20,78,30)]; awb.title=@"Lock WB"; awb.bezelStyle=NSBezelStyleRounded; awb.target=b; awb.action=@selector(awb:); [cv addSubview:awb];
    NSButton*ph=[[NSButton alloc] initWithFrame:NSMakeRect(598,20,114,30)]; ph.title=@"Bayer: BGGR"; ph.bezelStyle=NSBezelStyleRounded; ph.target=b; ph.action=@selector(phase:); [cv addSubview:ph];
    NSTextField*cl=[NSTextField labelWithString:@"Contrast"]; cl.frame=NSMakeRect(720,25,56,20); [cv addSubview:cl];
    NSSlider*cs=[NSSlider sliderWithValue:1.15 minValue:0.6 maxValue:3.0 target:b action:@selector(contrast:)]; cs.frame=NSMakeRect(776,22,92,22); [cv addSubview:cs];
    NSTextField*lul=[NSTextField labelWithString:@"Luma"]; lul.frame=NSMakeRect(878,25,40,20); [cv addSubview:lul];
    NSSlider*lus=[NSSlider sliderWithValue:1.0 minValue:0.2 maxValue:3.0 target:b action:@selector(luma:)]; lus.frame=NSMakeRect(918,22,100,22); [cv addSubview:lus];
    NSTextField*sal=[NSTextField labelWithString:@"Sat"]; sal.frame=NSMakeRect(1026,25,28,20); [cv addSubview:sal];
    NSSlider*sas=[NSSlider sliderWithValue:1.0 minValue:0.0 maxValue:3.0 target:b action:@selector(sat:)]; sas.frame=NSMakeRect(1054,22,100,22); [cv addSubview:sas];
    NSTextField*st=[NSTextField labelWithString:@"Connecting…"]; st.frame=NSMakeRect(1164,25,226,20); [cv addSubview:st]; r.status=st; g_status_label=st;

    // continuous auto white-balance (white-patch, temporally smoothed)
    [NSTimer scheduledTimerWithTimeInterval:0.4 target:b selector:@selector(awbAuto:) userInfo:nil repeats:YES];
    // Space = capture (physical touch button can't be read via libusb on macOS - INT EP 0x83 never delivers)
    [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskKeyDown handler:^NSEvent*(NSEvent*e){
        if(!e.isARepeat && [e.charactersIgnoringModifiers isEqualToString:@" "]){ g_capture_req=1; return nil; }
        return e; }];

    [win makeKeyAndOrderFront:nil]; [app activateIgnoringOtherApps:YES]; [app run]; g_run=0;
  }
  return 0;
}
