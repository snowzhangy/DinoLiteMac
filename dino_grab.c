// dino_grab.c - headless CLI grabber for the Dino-Lite AM411T (AnMo a168:0615)
// Sonix SN9C201 bridge + Micron MT9M111 sensor. Driver protocol ported from Linux gspca sn9c20x.
// Streams RAW Bayer (BGGR8) SXGA 1280x1023 over isochronous EP 0x81.
//
//   cc dino_grab.c -o dino_grab -I/opt/homebrew/include -L/opt/homebrew/lib -lusb-1.0 \
//        -Wl,-rpath,/opt/homebrew/lib
//   ./dino_grab            # save 3 PPM frames (+ raw PGM) to /tmp
//   ./dino_grab - | ffplay -f rawvideo -pixel_format rgb24 -video_size 1280x1023 -i -
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define VID 0xA168
#define PID 0x0615
#define IFACE 0
#define ALT 8
#define EP_ISO 0x81
#define W 1280
#define H 1024                     // sensor window height (for the window registers)
#define CH 1023                    // rows the bridge actually emits at SXGA
#define FRAME_BYTES (W*CH)         // BGGR8, 1 byte/pixel

#define I2C_ADDR 0x5d
#define I2C_INTF 0x81

static libusb_device_handle *dev;
static int g_err = 0;

// ---- transport (gspca reg_w/reg_r/i2c_w) ----
static void reg_w(uint16_t reg, const uint8_t *buf, int len){
    if(g_err) return;
    int r = libusb_control_transfer(dev,
        LIBUSB_ENDPOINT_OUT|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_RECIPIENT_INTERFACE,
        0x08, reg, 0x00, (unsigned char*)buf, len, 500);
    if(r!=len){ fprintf(stderr,"reg_w %04x failed %d\n",reg,r); g_err=r; }
}
static void reg_w1(uint16_t reg, uint8_t v){ reg_w(reg,&v,1); }
static int reg_r(uint16_t reg, uint8_t *buf, int len){
    if(g_err) return -1;
    int r = libusb_control_transfer(dev,
        LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_RECIPIENT_INTERFACE,
        0x00, reg, 0x00, buf, len, 500);
    if(r<0){ fprintf(stderr,"reg_r %04x failed %d\n",reg,r); g_err=r; }
    return r;
}
static void i2c_w(const uint8_t row[8]){
    uint8_t s[1];
    reg_w(0x10c0,row,8);
    for(int i=0;i<5;i++){
        if(reg_r(0x10c0,s,1)<0) return;
        if(s[0]&0x04){ if(s[0]&0x08){fprintf(stderr,"i2c err reg %02x\n",row[2]); g_err=-1;} return; }
        usleep(10000);
    }
    fprintf(stderr,"i2c reg %02x no response\n",row[2]);
}
static void i2c_w2(uint8_t reg, uint16_t val){
    uint8_t row[8]={ I2C_INTF|(3<<4), I2C_ADDR, reg, val>>8, val&0xff, 0,0, 0x10 };
    i2c_w(row);
}

// ---- init tables (from gspca sn9c20x.c) ----
static const uint16_t bridge_init[][2] = {
    {0x1000,0x78},{0x1001,0x40},{0x1002,0x1c},{0x1020,0x80},{0x1061,0x01},
    {0x1067,0x40},{0x1068,0x30},{0x1069,0x20},{0x106a,0x10},{0x106b,0x08},
    {0x1188,0x87},{0x11a1,0x00},{0x11a2,0x00},{0x11a3,0x6a},{0x11a4,0x50},
    {0x11ab,0x00},{0x11ac,0x00},{0x11ad,0x50},{0x11ae,0x3c},{0x118a,0x04},
    {0x0395,0x04},{0x11b8,0x3a},{0x118b,0x0e},{0x10f7,0x05},{0x10f8,0x14},
    {0x10fa,0xff},{0x10f9,0x00},{0x11ba,0x0a},{0x11a5,0x2d},{0x11a6,0x2d},
    {0x11a7,0x3a},{0x11a8,0x05},{0x11a9,0x04},{0x11aa,0x3f},{0x11af,0x28},
    {0x11b0,0xd8},{0x11b1,0x14},{0x11b2,0xec},{0x11b3,0x32},{0x11b4,0xdd},
    {0x11b5,0x32},{0x11b6,0xdd},{0x10e0,0x2c},{0x11bc,0x40},{0x11bd,0x01},
    {0x11be,0xf0},{0x11bf,0x00},{0x118c,0x1f},{0x118d,0x1f},{0x118e,0x1f},
    {0x118f,0x1f},{0x1180,0x01},{0x1181,0x00},{0x1182,0x01},{0x1183,0x00},
    {0x1184,0x50},{0x1185,0x80},{0x1007,0x00}
};
static const uint16_t mt9m111_init[][2] = {
    {0xf0,0x0000},{0x0d,0x0021},{0x0d,0x0008},{0xf0,0x0001},{0x3a,0x4300},
    {0x9b,0x4300},{0x06,0x708e},{0xf0,0x0002},{0x2e,0x0a1e},{0xf0,0x0000},
};

static void dino_init(void){
    uint8_t v;
    for(unsigned i=0;i<sizeof(bridge_init)/sizeof(bridge_init[0]);i++){
        v = bridge_init[i][1];
        reg_w(bridge_init[i][0], &v, 1);
    }
    reg_w1(0x1006, 0x20);                                   // LED on
    uint8_t i2c_init[9]={0x80,I2C_ADDR,0,0,0,0,0,0,0x03};
    reg_w(0x10c0, i2c_init, 9);
    for(unsigned i=0;i<sizeof(mt9m111_init)/sizeof(mt9m111_init[0]);i++)
        i2c_w2(mt9m111_init[i][0], mt9m111_init[i][1]);
}

static void dino_start(void){
    int hstart=0, vstart=2;
    // configure_sensor_output (MT9M111, SXGA 1280x1024)
    i2c_w2(0xf0,0x0002); i2c_w2(0xc8,0x970b); i2c_w2(0xf0,0x0000);
    uint8_t clr[5]={0, (W>>2)&0xff, 0, (H>>1)&0xff, (uint8_t)(((W>>10)&0x01)|((H>>8)&0x6))};
    reg_w(0x10fb, clr, 5);
    uint8_t hw[6]={(uint8_t)hstart,0,(uint8_t)vstart,0,(W>>4)&0xff,(H>>3)&0xff};
    reg_w(0x1180, hw, 6);
    reg_w1(0x1189, 0xc0);     // scale 1280x1024 (SXGA)
    reg_w1(0x10e0, 0x2d);     // fmt RAW BGGR8
    // MT9M111 runs its own hardware AE/AGC - leave exposure/gain to it
    reg_w1(0x1007, 0x20);     // stream on
    reg_w1(0x1061, 0x03);
}
static void dino_stop(void){ reg_w1(0x1007,0x00); reg_w1(0x1061,0x01); reg_w1(0x1006,0x00); }

// ---- frame assembler ----
static const uint8_t FHDR[6]={0xff,0xff,0x00,0xc4,0xc4,0x96};
static uint8_t framebuf[FRAME_BYTES+W*4];
static int fill=0;
static int frames_done=0, target=30, streaming=1;
static uint8_t *latest=NULL;

static void emit_frame(void){
    if(fill < FRAME_BYTES) { fill=0; return; }
    if(!latest) latest=malloc(FRAME_BYTES);
    memcpy(latest, framebuf, FRAME_BYTES);
    frames_done++;
    fill=0;
}
static void feed(uint8_t *data, int len){
    if(len>=64 && memcmp(data,FHDR,6)==0){    // header => previous frame complete
        emit_frame();
        data+=64; len-=64;
    }
    if(len<=0) return;
    if(fill+len > (int)sizeof(framebuf)) len = sizeof(framebuf)-fill;
    memcpy(framebuf+fill, data, len); fill+=len;
}
static void LIBUSB_CALL cb(struct libusb_transfer *t){
    if(streaming){
        for(int i=0;i<t->num_iso_packets;i++){
            struct libusb_iso_packet_descriptor *d=&t->iso_packet_desc[i];
            if(d->status==LIBUSB_TRANSFER_COMPLETED && d->actual_length>0)
                feed(libusb_get_iso_packet_buffer_simple(t,i), d->actual_length);
        }
        if(frames_done<target) { libusb_submit_transfer(t); return; }
        streaming=0;
    }
}

// ---- debayer BGGR8 -> RGB24 ----
static inline int px(uint8_t*b,int x,int y){
    if(x<0)x=0; if(x>=W)x=W-1; if(y<0)y=0; if(y>=CH)y=CH-1;
    return b[y*W+x];
}
static void debayer(uint8_t*raw, uint8_t*rgb){
    for(int y=0;y<CH;y++) for(int x=0;x<W;x++){
        int R,G,B, c=raw[y*W+x];
        int ye=!(y&1), xe=!(x&1);                       // BGGR: (even,even)=B (odd,odd)=R
        if(ye&&xe){ B=c; G=(px(raw,x-1,y)+px(raw,x+1,y)+px(raw,x,y-1)+px(raw,x,y+1))/4;
                    R=(px(raw,x-1,y-1)+px(raw,x+1,y-1)+px(raw,x-1,y+1)+px(raw,x+1,y+1))/4; }
        else if(!ye&&!xe){ R=c; G=(px(raw,x-1,y)+px(raw,x+1,y)+px(raw,x,y-1)+px(raw,x,y+1))/4;
                    B=(px(raw,x-1,y-1)+px(raw,x+1,y-1)+px(raw,x-1,y+1)+px(raw,x+1,y+1))/4; }
        else if(ye&&!xe){ G=c; B=(px(raw,x-1,y)+px(raw,x+1,y))/2; R=(px(raw,x,y-1)+px(raw,x,y+1))/2; }
        else { G=c; R=(px(raw,x-1,y)+px(raw,x+1,y))/2; B=(px(raw,x,y-1)+px(raw,x,y+1))/2; }
        rgb[(y*W+x)*3+0]=R; rgb[(y*W+x)*3+1]=G; rgb[(y*W+x)*3+2]=B;
    }
}

int main(int argc, char**argv){
    int to_stdout = (argc>1 && strcmp(argv[1],"-")==0);
    if(to_stdout) target=100000;          // stream until the pipe closes
    libusb_context *ctx=NULL;
    if(libusb_init(&ctx)){ fprintf(stderr,"libusb_init fail\n"); return 1; }
    dev=libusb_open_device_with_vid_pid(ctx,VID,PID);
    if(!dev){ fprintf(stderr,"device %04x:%04x not found\n",VID,PID); return 1; }
    libusb_set_auto_detach_kernel_driver(dev,1);
    libusb_reset_device(dev);   // recover if a prior process was SIGKILLed mid-claim
    if(libusb_claim_interface(dev,IFACE)){ fprintf(stderr,"claim fail\n"); return 1; }

    dino_init();
    dino_start();
    if(g_err){ fprintf(stderr,"init/start error %d\n",g_err); }
    if(libusb_set_interface_alt_setting(dev,IFACE,ALT)){ fprintf(stderr,"set alt %d fail\n",ALT); }

    int NT=8, NP=64, PSZ=5120;
    struct libusb_transfer *tr[16];
    for(int i=0;i<NT;i++){
        tr[i]=libusb_alloc_transfer(NP);
        uint8_t *buf=malloc(NP*PSZ);
        libusb_fill_iso_transfer(tr[i],dev,EP_ISO,buf,NP*PSZ,NP,cb,NULL,0);
        libusb_set_iso_packet_lengths(tr[i],PSZ);
        if(libusb_submit_transfer(tr[i])) fprintf(stderr,"submit %d fail\n",i);
    }
    fprintf(stderr,"streaming %dx%d RAW (alt %d)...\n",W,CH,ALT);

    uint8_t *rgb=malloc(W*CH*3);
    int saved=0;
    while(streaming){
        struct timeval tv={1,0};
        libusb_handle_events_timeout(ctx,&tv);
        if(to_stdout && latest){
            debayer(latest,rgb); fwrite(rgb,1,W*CH*3,stdout); fflush(stdout);
            free(latest); latest=NULL;
        } else if(!to_stdout && latest && frames_done>=5 && saved<3){
            debayer(latest,rgb);                    // skip first frames (AE settle), save 3
            char fn[64]; snprintf(fn,sizeof fn,"/tmp/dino_%d.ppm",saved);
            FILE*f=fopen(fn,"wb"); fprintf(f,"P6\n%d %d\n255\n",W,CH);
            fwrite(rgb,1,W*CH*3,f); fclose(f);
            char gn[64]; snprintf(gn,sizeof gn,"/tmp/dino_raw_%d.pgm",saved);
            FILE*g=fopen(gn,"wb"); fprintf(g,"P5\n%d %d\n255\n",W,CH);
            fwrite(latest,1,W*CH,g); fclose(g);
            fprintf(stderr,"saved %s + raw PGM\n",fn);
            free(latest); latest=NULL; saved++;
            if(saved>=3) break;
        }
    }
    streaming=0;
    for(int i=0;i<NT;i++) libusb_cancel_transfer(tr[i]);
    struct timeval tv={0,200000}; libusb_handle_events_timeout(ctx,&tv);
    dino_stop();
    libusb_release_interface(dev,IFACE);
    libusb_close(dev); libusb_exit(ctx);
    fprintf(stderr,"done. frames=%d saved=%d\n",frames_done,saved);
    return 0;
}
