// dino_shot.c - single-frame grab for the Dino-Lite AM411T, sharing dino_metal's color path.
// Streams RAW Bayer SXGA, lets AE settle, runs green-ref AWB + auto-levels, then debayers ONE
// frame (the PROVEN true-color path ported from librepods/tools/dinolite_snap.c) and writes a PNG.
// Tuning params are CLI args so the color pipeline can be swept without recompiling.
//
//   cc dino_shot.c -o dino_shot -I/opt/homebrew/include -L/opt/homebrew/lib -lusb-1.0 \
//        -Wl,-rpath,/opt/homebrew/lib -framework CoreGraphics -framework ImageIO -framework CoreFoundation
//   ./dino_shot [out.png] [sat] [gamma] [phase] [exposure] [cblur]
//     out.png   default /tmp/dino_shot.png
//     sat       saturation        (default 0.25 - the proven value; >0.4 oversaturates)
//     gamma     tone gamma        (default 2.15)
//     phase     Bayer 0..3        (default 0 = GBRG, the layout of the SXGA payload)
//     exposure  pin shutter 0x09  (default 0 = software AE on; >0 pins AE off)
//     cblur     chroma-only blur  (default 0; merges CMYK halftone dots -> true color)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <libusb-1.0/libusb.h>
#include <ImageIO/ImageIO.h>
#include <CoreGraphics/CoreGraphics.h>

#define VID 0xA168
#define PID 0x0615
#define IFACE 0
#define ALT 8
#define EP_ISO 0x81
#define W 1280
#define H 1024
#define CH 1023
#define FRAME_BYTES (W*CH)
#define I2C_ADDR 0x5d
#define I2C_INTF 0x81

static libusb_device_handle *dev; static int g_err=0;
// color params (set from argv). NB: no contrast term - the proven path has none.
static float g_gR=1.0f,g_gG=1.0f,g_gB=1.0f, g_black=7.0f,g_white=153.0f,g_gamma=2.15f,g_sat=0.25f;
static float g_tintR=1.0f,g_tintB=1.0f;   // warm/cool trim applied on top of AWB (B<1 = warmer)
static float g_contrast=1.15f;            // contrast about mid-gray (vendor DinoCapture = 1.15)
static int g_ae_target=60;                // AE luma target (vendor DinoCapture = 60)
static int g_phase=3, g_exp=0x180, g_hdr_luma=-1, g_hdr_ready=0, g_ae_off=0;

// ---------- transport ----------
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
    reg_w1(0x1006,0x20);
    uint8_t ii[9]={0x80,I2C_ADDR,0,0,0,0,0,0,0x03}; reg_w(0x10c0,ii,9);
    for(unsigned i=0;i<sizeof(mt9m111_init)/sizeof(mt9m111_init[0]);i++) i2c_w2(mt9m111_init[i][0],mt9m111_init[i][1]); }
static void dino_start(void){ int hs=0,vs=2;
    i2c_w2(0xf0,0x0002); i2c_w2(0xc8,0x970b); i2c_w2(0xf0,0x0000);
    uint8_t clr[5]={0,(W>>2)&0xff,0,(H>>1)&0xff,(uint8_t)(((W>>10)&1)|((H>>8)&6))}; reg_w(0x10fb,clr,5);
    uint8_t hw[6]={(uint8_t)hs,0,(uint8_t)vs,0,(W>>4)&0xff,(H>>3)&0xff}; reg_w(0x1180,hw,6);
    reg_w1(0x1189,0xc0); reg_w1(0x10e0,0x2d);
    i2c_w2(0xf0,0x0000); i2c_w2(0x09,(uint16_t)g_exp); i2c_w2(0x2c,0x0040); i2c_w2(0x2d,0x0040); i2c_w2(0x2e,0x0040); i2c_w2(0x2f,0x0040); i2c_w2(0xf0,0x0000);
    reg_w1(0x1007,0x20); reg_w1(0x1061,0x03); }
static void dino_stop(void){ reg_w1(0x1007,0x00); reg_w1(0x1061,0x01); reg_w1(0x1006,0x00); }

// ---------- frame assembler ----------
static const uint8_t FHDR[6]={0xff,0xff,0x00,0xc4,0xc4,0x96};
static uint8_t framebuf[FRAME_BYTES+W*4]; static int fill=0;
static uint8_t latest[FRAME_BYTES]; static int frames=0;
static int sonix_header_luma(const uint8_t*d,int len){ if(len<64)return -1; int avg=0;
    avg+=((d[35]>>2)&3)|(d[20]<<2)|(d[19]<<10); avg+=((d[35]>>4)&3)|(d[22]<<2)|(d[21]<<10);
    avg+=((d[35]>>6)&3)|(d[24]<<2)|(d[23]<<10); avg+=(d[36]&3)|(d[26]<<2)|(d[25]<<10);
    avg+=((d[36]>>2)&3)|(d[28]<<2)|(d[27]<<10); avg+=((d[36]>>4)&3)|(d[30]<<2)|(d[29]<<10);
    avg+=((d[36]>>6)&3)|(d[32]<<2)|(d[31]<<10); avg+=((d[44]>>4)&3)|(d[34]<<2)|(d[33]<<10); return avg>>9; }
static void emit(void){ if(fill>=FRAME_BYTES){ memcpy(latest,framebuf,FRAME_BYTES); frames++; } fill=0; }
static void feed(uint8_t*d,int len){ if(len>=64&&memcmp(d,FHDR,6)==0){emit(); int lum=sonix_header_luma(d,len); if(lum>=0){g_hdr_luma=lum;g_hdr_ready=1;} d+=64;len-=64;} if(len<=0)return;
    if(fill+len>(int)sizeof(framebuf))len=sizeof(framebuf)-fill; memcpy(framebuf+fill,d,len); fill+=len; }
static void LIBUSB_CALL iso_cb(struct libusb_transfer*t){
    for(int i=0;i<t->num_iso_packets;i++){ struct libusb_iso_packet_descriptor*p=&t->iso_packet_desc[i];
        if(p->status==LIBUSB_TRANSFER_COMPLETED&&p->actual_length>0) feed(libusb_get_iso_packet_buffer_simple(t,i),p->actual_length); }
    libusb_submit_transfer(t); }

// ---------- color pipeline (PROVEN path, mirrors librepods dinolite_snap.c) ----------
static inline int raw_px(const uint8_t*raw,int x,int y){ if(x<0)x=0;if(x>=W)x=W-1;if(y<0)y=0;if(y>=CH)y=CH-1;return raw[y*W+x]; }
static inline int iclp(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);}
static inline float fclp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
// tone: auto black/white levels (white-point fixes exposure/wash) -> sRGB encode -> contrast.
static inline float srgbf(float v){ v=fclp(v,0.0f,1.0f); return v<=0.0031308f?12.92f*v:1.055f*powf(v,1.0f/2.4f)-0.055f; }
static inline uint8_t tone(float v){ float x=fclp((v-g_black)/(g_white-g_black),0.0f,1.0f);
    x=srgbf(x); x=fclp((x-0.5f)*g_contrast+0.5f,0.0f,1.0f);
    return (uint8_t)iclp((int)lrintf(x*255.0f),0,255); }
// 2x2 BLOCK chroma. Inherently soft (one color per Bayer quad) - that is the "soft chroma" that
// dissolves lateral-CA fringe; the per-pixel sharpness comes from local_luma_detail() instead.
// phase: 0 GBRG (the SXGA payload layout) 1 GRBG 2 RGGB 3 BGGR. Keep in sync with the AWB sampler.
static inline void cell_rgb(const uint8_t*raw,int x,int y,int phase,float*r,float*g,float*b){
    int bx=x&~1, by=y&~1; if(bx+1>=W)bx=W-2; if(by+1>=CH)by=(CH-2)&~1;
    float p00=raw[by*W+bx], p01=raw[by*W+bx+1], p10=raw[(by+1)*W+bx], p11=raw[(by+1)*W+bx+1];
    switch(phase){
        case 0: *g=(p00+p11)*0.5f; *b=p01; *r=p10; break;   // GBRG (row even: G B, row odd: R G)
        case 1: *g=(p01+p10)*0.5f; *r=p00; *b=p11; break;   // GRBG
        case 2: *r=p00; *g=(p01+p10)*0.5f; *b=p11; break;   // RGGB
        default:*b=p00; *g=(p01+p10)*0.5f; *r=p11; break;   // BGGR
    }
}
// crisp per-pixel luma (center + 4-neighbour cross, weights sum to 1.0).
static inline float local_luma_detail(const uint8_t*raw,int x,int y){
    return (float)raw_px(raw,x,y)*0.45f +
           (float)(raw_px(raw,x-1,y)+raw_px(raw,x+1,y)+raw_px(raw,x,y-1)+raw_px(raw,x,y+1))*0.1375f; }
static int hist_pct(const unsigned hist[256],unsigned total,float q){ unsigned target=(unsigned)((float)total*q+0.5f),acc=0;
    for(int i=0;i<256;i++){acc+=hist[i]; if(acc>=target)return i;} return 255; }
// AWB (vendor/committed path): white-patch on the BRIGHTEST cells, balanced to the MAX channel
// so whites go neutral and any cast (green/magenta) is removed. Run ONCE on the settled frame.
static void compute_awb(const uint8_t*raw){
    unsigned hist[256]={0},total=0; int mx=1;
    for(int y=0;y+1<CH;y+=2)for(int x=0;x+1<W;x+=2){
        float Rf,Gf,Bf; cell_rgb(raw,x,y,g_phase,&Rf,&Gf,&Bf);
        int lum=((int)Rf+2*(int)Gf+(int)Bf)>>2; lum=iclp(lum,0,255); hist[lum]++; total++; if(lum>mx)mx=lum;
    }
    int thr=mx*45/100; if(thr<20)thr=20;
    long long sR=0,sG=0,sB=0,n=0;
    for(int y=0;y+1<CH;y+=2)for(int x=0;x+1<W;x+=2){
        float Rf,Gf,Bf; cell_rgb(raw,x,y,g_phase,&Rf,&Gf,&Bf);
        int R=(int)Rf,G=(int)Gf,B=(int)Bf;
        int lum=(R+2*G+B)>>2;
        if(lum>=thr && R<248 && G<248 && B<248){ sR+=R;sG+=G;sB+=B;n++; }
    }
    // auto black/white levels: white-point at the 99.7th pct maps the brightest cells to display white
    int black=hist_pct(hist,total,0.006f), white=hist_pct(hist,total,0.997f);
    if(white-black<55) white=iclp(black+55,0,255);
    g_black=(float)iclp(black-1,0,64);
    g_white=fclp((float)white, g_black+48.0f, 255.0f);
    if(n<50){ g_gR=g_gG=g_gB=1.0f; return; }
    double mR=(double)sR/n+1e-6,mG=(double)sG/n+1e-6,mB=(double)sB/n+1e-6;
    double ref=mR; if(mG>ref)ref=mG; if(mB>ref)ref=mB;
    g_gR=fclp((float)(ref/mR),0.5f,4.0f);
    g_gG=fclp((float)(ref/mG),0.5f,4.0f);
    g_gB=fclp((float)(ref/mB),0.5f,4.0f);
}
static void compute_ae(const uint8_t*raw){
    static int locked=0;
    int lum=g_hdr_luma;
    if(!g_hdr_ready||lum<=0||lum>255){ long long s=0,n=0;
        for(int y=0;y<CH;y+=8) for(int x=0;x<W;x+=8){ s+=raw[y*W+x]; n++; } lum=(int)(s/n); }
    g_hdr_ready=0;
    int T=g_ae_target;
    if(locked){ if(lum>=T-14&&lum<=T+14) return; locked=0; }   // hysteresis around the target band
    int dir=0;
    if(lum<T-6)dir=1; else if(lum>T+6)dir=-1; else { locked=1; return; }
    int step=(lum<T-30||lum>T+40)?14:6;
    int next=iclp(g_exp+dir*step,0x40,0x420);
    if(next!=g_exp){ g_exp=next; i2c_w2(0xf0,0x0000); i2c_w2(0x09,(uint16_t)g_exp); i2c_w2(0xf0,0x0000); }
}
static float chR[FRAME_BYTES],chG[FRAME_BYTES],chB[FRAME_BYTES];   // WB-gained chroma planes
static float Yp[FRAME_BYTES];                                      // luma plane
static float tmpp[FRAME_BYTES];
static int g_cblur=0;   // chroma-ONLY blur radius: merges CMYK halftone dots into solid color
static int g_lblur=0;   // LUMA blur radius: soft-focus -> dissolves dark print-outline fringe on text edges
// separable box blur of one plane (horizontal then vertical), radius r, edge-clamped.
static void box_blur(float*p,int r){
    if(r<=0)return; float inv=1.0f/(2*r+1);
    for(int y=0;y<CH;y++){ float*row=p+y*W; float s=0; for(int k=-r;k<=r;k++)s+=row[iclp(k,0,W-1)];
        for(int x=0;x<W;x++){ tmpp[y*W+x]=s*inv; s+=row[iclp(x+r+1,0,W-1)]-row[iclp(x-r,0,W-1)]; } }
    for(int x=0;x<W;x++){ float s=0; for(int k=-r;k<=r;k++)s+=tmpp[iclp(k,0,CH-1)*W+x];
        for(int y=0;y<CH;y++){ p[y*W+x]=s*inv; s+=tmpp[iclp(y+r+1,0,CH-1)*W+x]-tmpp[iclp(y-r,0,CH-1)*W+x]; } }
}
// debayer: sharp luma Y carries brightness; the (optionally blurred) block chroma RATIO is
// re-keyed onto Y, then saturation pulls toward gray. White paper -> neutral ratio -> stays
// white. Chroma blur merges halftone dots so printed reds read true-red, not magenta. Luma stays
// per-pixel sharp, so edges never go black. (proven raw_to_rgba + chroma low-pass)
static void cpu_debayer(const uint8_t*raw,uint8_t*rgba){
    for(int y=0;y<CH;y++)for(int x=0;x<W;x++){ int i=y*W+x; float R,G,B; cell_rgb(raw,x,y,g_phase,&R,&G,&B);
        chR[i]=R*g_gR; chG[i]=G*g_gG; chB[i]=B*g_gB; Yp[i]=local_luma_detail(raw,x,y); }
    if(g_cblur>0){ box_blur(chR,g_cblur); box_blur(chG,g_cblur); box_blur(chB,g_cblur); }
    if(g_lblur>0) box_blur(Yp,g_lblur);   // soft-focus the luma: thin dark edge lines average out
    for(int y=0;y<CH;y++)for(int x=0;x<W;x++){ int i=y*W+x;
        float Y=Yp[i];
        float R=chR[i],G=chG[i],B=chB[i];
        float chroma_y=(R+2.0f*G+B)*0.25f; if(chroma_y<1.0f)chroma_y=1.0f;
        R=Y*(R/chroma_y); G=Y*(G/chroma_y); B=Y*(B/chroma_y);
        float luma=(R+G+B)/3.0f;
        R=luma+(R-luma)*g_sat; G=luma+(G-luma)*g_sat; B=luma+(B-luma)*g_sat;
        R*=g_tintR; B*=g_tintB;                 // warm/cool trim on FINAL rgb (not normalized away)
        uint8_t*p=rgba+i*4; p[0]=tone(R);p[1]=tone(G);p[2]=tone(B);p[3]=255;
    }
}
static int save_png(const char*path,const uint8_t*rgba){
    int ok=0; CGColorSpaceRef cs=CGColorSpaceCreateDeviceRGB();
    CGContextRef c=CGBitmapContextCreate(NULL,W,CH,8,W*4,cs,kCGImageAlphaNoneSkipLast|kCGBitmapByteOrder32Big);
    if(c){ memcpy(CGBitmapContextGetData(c),rgba,(size_t)W*CH*4); CGImageRef im=CGBitmapContextCreateImage(c);
        if(im){ CFStringRef p=CFStringCreateWithCString(NULL,path,kCFStringEncodingUTF8);
            CFURLRef url=CFURLCreateWithFileSystemPath(NULL,p,kCFURLPOSIXPathStyle,false);
            CGImageDestinationRef d=CGImageDestinationCreateWithURL(url,CFSTR("public.png"),1,NULL);
            if(d){ CGImageDestinationAddImage(d,im,NULL); ok=CGImageDestinationFinalize(d); CFRelease(d);}
            if(url)CFRelease(url); CFRelease(p); CGImageRelease(im);} CGContextRelease(c);}
    CGColorSpaceRelease(cs); return ok;
}

int main(int argc,char**argv){
    const char*out = argc>1?argv[1]:"/tmp/dino_shot.png";
    if(argc>2)g_sat=atof(argv[2]);
    if(argc>3)g_contrast=atof(argv[3]);   // contrast about mid-gray (vendor 1.15)
    if(argc>4)g_phase=atoi(argv[4])&3;
    if(argc>5){ int e=(int)strtol(argv[5],NULL,0); if(e>0){ g_exp=e; g_ae_off=1; } }  // exposure>0 pins AE off; 0=keep AE
    if(argc>6)g_cblur=atoi(argv[6]);                                                  // chroma-only blur radius
    if(argc>7)g_tintB=atof(argv[7]);                                                  // blue trim (<1 warmer)
    if(argc>8)g_tintR=atof(argv[8]);                                                  // red trim
    if(argc>9)g_lblur=atoi(argv[9]);                                                  // luma blur (soft focus)
    libusb_context*ctx=NULL;
    if(libusb_init(&ctx)){fprintf(stderr,"libusb_init fail\n");return 1;}
    dev=libusb_open_device_with_vid_pid(ctx,VID,PID);
    if(!dev){fprintf(stderr,"device %04x:%04x not found\n",VID,PID);return 1;}
    libusb_set_auto_detach_kernel_driver(dev,1);
    libusb_reset_device(dev);
    if(libusb_claim_interface(dev,IFACE)){fprintf(stderr,"claim fail\n");return 1;}
    dino_init(); dino_start();
    if(libusb_set_interface_alt_setting(dev,IFACE,ALT)) fprintf(stderr,"set alt fail\n");
    int NT=8,NP=64,PSZ=5120; struct libusb_transfer*tr[16];
    for(int i=0;i<NT;i++){ tr[i]=libusb_alloc_transfer(NP); uint8_t*b=malloc(NP*PSZ);
        libusb_fill_iso_transfer(tr[i],dev,EP_ISO,b,NP*PSZ,NP,iso_cb,NULL,0); libusb_set_iso_packet_lengths(tr[i],PSZ); libusb_submit_transfer(tr[i]); }
    // settle (~70 frames) with software AE every 4 frames; analyze color ONCE at the end.
    int last_ae=0, want=70;
    while(frames<want){ struct timeval tv={1,0}; libusb_handle_events_timeout(ctx,&tv);
        if(!g_ae_off && frames-last_ae>=4){ last_ae=frames; compute_ae(latest); } }
    compute_awb(latest);
    static uint8_t rgba[W*CH*4]; cpu_debayer(latest,rgba);
    int ok=save_png(out,rgba);
    { long long sraw=0; for(int i=0;i<FRAME_BYTES;i++)sraw+=latest[i];
      double rawmean=(double)sraw/FRAME_BYTES;
      long long sout=0; for(int i=0;i<W*CH;i++){uint8_t*p=rgba+i*4; sout+=(p[0]*299+p[1]*587+p[2]*114)/1000;}
      double outluma=(double)sout/(W*CH);
      fprintf(stderr,"LUMA: sensor-header(AE)=%d  raw-mean=%.0f  OUTPUT-image=%.0f  (vendor target 60)\n",
              g_hdr_luma,rawmean,outluma); }
    fprintf(stderr,"saved %s  exp=0x%03x WB R%.2f/G%.2f/B%.2f lvl %.0f/%.0f sat=%.2f gam=%.2f phase=%d frames=%d\n",
            out,g_exp,g_gR,g_gG,g_gB,g_black,g_white,g_sat,g_gamma,g_phase,frames);
    for(int i=0;i<NT;i++) libusb_cancel_transfer(tr[i]);
    struct timeval tvf={0,200000}; libusb_handle_events_timeout(ctx,&tvf);
    dino_stop(); libusb_release_interface(dev,IFACE); libusb_close(dev); libusb_exit(ctx);
    return ok?0:2;
}
