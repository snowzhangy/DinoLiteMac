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

static libusb_device_handle *dev; static libusb_context *ctx;
static int g_err=0, g_run=1; static pthread_t g_engine;
static volatile int g_led=0x20, g_led_applied=-1;  // 0x20=on, 0x00=off (single GPIO LED, no brightness)
static volatile int g_capture_req=0;
static volatile int g_gray=0;
static volatile int g_phase=3;        // Bayer phase 0=GBRG 1=GRBG 2=RGGB 3=BGGR
static volatile float g_contrast=1.80f;
static volatile float g_gR=1.0f, g_gG=1.0f, g_gB=1.0f;
static pthread_mutex_t lock=PTHREAD_MUTEX_INITIALIZER;
static uint8_t shared_raw[FRAME_BYTES]; static int shared_new=0;
static volatile int g_awb_n=14;   // AWB converge-then-hold counter (converges over N frames, then locks)

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
    reg_w1(0x1189,0xc0); reg_w1(0x10e0,0x2d);
    // MT9M111 runs its own hardware AE/AGC - do NOT write shutter(0x09)/gain(0x2c-0x2f) or it fights the hw loop and pumps
    reg_w1(0x1007,0x20); reg_w1(0x1061,0x03); }
static void dino_stop(void){ reg_w1(0x1007,0x00); reg_w1(0x1061,0x01); }

// ---------- frame assembler ----------
static const uint8_t FHDR[6]={0xff,0xff,0x00,0xc4,0xc4,0x96};
static uint8_t framebuf[FRAME_BYTES+W*4]; static int fill=0;
static void emit(void){ if(fill>=FRAME_BYTES){ pthread_mutex_lock(&lock); memcpy(shared_raw,framebuf,FRAME_BYTES); shared_new=1; pthread_mutex_unlock(&lock);} fill=0; }
static void feed(uint8_t*d,int len){ if(len>=64&&memcmp(d,FHDR,6)==0){emit();d+=64;len-=64;} if(len<=0)return;
    if(fill+len>(int)sizeof(framebuf))len=sizeof(framebuf)-fill; memcpy(framebuf+fill,d,len); fill+=len; }
static void LIBUSB_CALL iso_cb(struct libusb_transfer*t){ if(!g_run)return;
    for(int i=0;i<t->num_iso_packets;i++){ struct libusb_iso_packet_descriptor*p=&t->iso_packet_desc[i];
        if(p->status==LIBUSB_TRANSFER_COMPLETED&&p->actual_length>0) feed(libusb_get_iso_packet_buffer_simple(t,i),p->actual_length); }
    libusb_submit_transfer(t); }
static void* engine(void*a){
    if(libusb_init(&ctx)){fprintf(stderr,"libusb_init fail\n");return 0;}
    dev=libusb_open_device_with_vid_pid(ctx,VID,PID);
    if(!dev){fprintf(stderr,"device not found\n");return 0;}
    libusb_set_auto_detach_kernel_driver(dev,1);
    if(libusb_claim_interface(dev,IFACE)){fprintf(stderr,"claim fail\n");return 0;}
    dino_init(); dino_start(); libusb_set_interface_alt_setting(dev,IFACE,ALT);
    int NT=10,NP=64,PSZ=5120; struct libusb_transfer*tr[16];
    for(int i=0;i<NT;i++){ tr[i]=libusb_alloc_transfer(NP); uint8_t*b=malloc(NP*PSZ);
        libusb_fill_iso_transfer(tr[i],dev,EP_ISO,b,NP*PSZ,NP,iso_cb,NULL,0); libusb_set_iso_packet_lengths(tr[i],PSZ); libusb_submit_transfer(tr[i]); }
    // The physical touch button reports only on INT EP 0x83, which macOS libusb never delivers,
    // so capture is triggered from the UI (Capture button / Spacebar) instead.
    while(g_run){ struct timeval tv={0,20000}; libusb_handle_events_timeout(ctx,&tv);
        if(g_led!=g_led_applied){ reg_w1(0x1006,g_led); g_led_applied=g_led; } }
    for(int i=0;i<NT;i++) libusb_cancel_transfer(tr[i]);
    struct timeval tvf={0,100000}; libusb_handle_events_timeout(ctx,&tvf);
    dino_stop(); reg_w1(0x1006,0x00);   // turn LED off on exit
    libusb_release_interface(dev,IFACE); libusb_close(dev); libusb_exit(ctx); return 0; }

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
typedef struct { int iw,ih,gray,phase; float gR,gG,gB,contrast; } Uniforms;
static NSString *kShader = @
"#include <metal_stdlib>\n"
"using namespace metal;\n"
"struct Uniforms { int iw; int ih; int gray; int phase; float gR; float gG; float gB; float contrast; };\n"
"struct VOut { float4 pos [[position]]; float2 uv; };\n"
"vertex VOut vmain(uint vid [[vertex_id]]){\n"
"  float2 p[3]={float2(-1,-1),float2(3,-1),float2(-1,3)};\n"
"  VOut o; o.pos=float4(p[vid],0,1); float2 t=p[vid]*0.5+0.5; o.uv=float2(t.x,1.0-t.y); return o; }\n"
"static inline float rd(texture2d<float,access::read> b,int x,int y,int W,int H){\n"
"  x=clamp(x,0,W-1); y=clamp(y,0,H-1); return b.read(uint2(x,y)).r*255.0; }\n"
"fragment float4 fmain(VOut in [[stage_in]], texture2d<float,access::read> bayer [[texture(0)]], constant Uniforms& u [[buffer(0)]]){\n"
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
"  return float4(c,1.0); }\n";

@interface Renderer : NSObject <MTKViewDelegate>
@property(strong) id<MTLDevice> dev;
@property(strong) id<MTLCommandQueue> q;
@property(strong) id<MTLRenderPipelineState> pso;
@property(strong) id<MTLTexture> bayerTex;
@property(strong) NSTextField *status;
@property(assign) uint8_t *upbuf;   // staging copy of latest raw
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
    MTLTextureDescriptor*td=[MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:W height:CH mipmapped:NO];
    td.usage=MTLTextureUsageShaderRead;
    self.bayerTex=[self.dev newTextureWithDescriptor:td];
    self.upbuf=malloc(FRAME_BYTES);
    return self;
}
-(void)mtkView:(MTKView*)v drawableSizeWillChange:(CGSize)s{}
-(void)drawInMTKView:(MTKView*)view{
    int have=0;
    pthread_mutex_lock(&lock);
    if(shared_new){ memcpy(self.upbuf,shared_raw,FRAME_BYTES); shared_new=0; have=1; }
    pthread_mutex_unlock(&lock);
    if(have) [self.bayerTex replaceRegion:MTLRegionMake2D(0,0,W,CH) mipmapLevel:0 withBytes:self.upbuf bytesPerRow:W];
    id<MTLCommandBuffer> cb=[self.q commandBuffer];
    MTLRenderPassDescriptor*rp=view.currentRenderPassDescriptor;
    if(rp){
        id<MTLRenderCommandEncoder> en=[cb renderCommandEncoderWithDescriptor:rp];
        [en setRenderPipelineState:self.pso];
        Uniforms u={W,CH,g_gray,g_phase,g_gR,g_gG,g_gB,g_contrast};
        [en setFragmentTexture:self.bayerTex atIndex:0];
        [en setFragmentBytes:&u length:sizeof(u) atIndex:0];
        [en drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [en endEncoding];
        [cb presentDrawable:view.currentDrawable];
    }
    [cb commit];
    if(g_capture_req){ g_capture_req=0; [self saveShot]; }
}
-(BOOL)savePNG:(NSString*)path{
    static uint8_t rgba[W*CH*4];
    pthread_mutex_lock(&lock); cpu_debayer(shared_raw,rgba,g_gR,g_gG,g_gB,g_gray,g_phase,g_contrast); pthread_mutex_unlock(&lock);
    BOOL ok=NO; CGColorSpaceRef cs=CGColorSpaceCreateDeviceRGB();
    CGContextRef c=CGBitmapContextCreate(NULL,W,CH,8,W*4,cs,kCGImageAlphaNoneSkipLast|kCGBitmapByteOrderDefault);
    if(c){ memcpy(CGBitmapContextGetData(c),rgba,(size_t)W*CH*4); CGImageRef im=CGBitmapContextCreateImage(c);
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
-(void)awb:(id)s{ g_awb_n=14; self.r.status.stringValue=@"Auto WB…"; }   // re-converge then hold
-(void)awbAuto:(id)s{ if(g_awb_n>0){ g_awb_n--; [self.r awbTick];
    if(g_awb_n==0) self.r.status.stringValue=[NSString stringWithFormat:@"WB locked R%.2f G%.2f B%.2f",g_gR,g_gG,g_gB]; } }
-(void)phase:(NSButton*)s{ g_phase=(g_phase+1)&3; const char*nm[4]={"GBRG","GRBG","RGGB","BGGR"};
    s.title=[NSString stringWithFormat:@"Bayer: %s",nm[g_phase]]; g_awb_n=14; }   // re-converge WB for new phase
-(void)contrast:(NSSlider*)s{ g_contrast=s.floatValue; }
@end

@interface AppDelegate:NSObject <NSApplicationDelegate> @end
@implementation AppDelegate
-(BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)a{ return YES; }
-(void)applicationWillTerminate:(NSNotification*)n{ g_run=0; pthread_join(g_engine,NULL); }  // engine cleanup turns LED off
@end

int main(){
  @autoreleasepool{
    pthread_create(&g_engine,NULL,engine,NULL);
    NSApplication*app=[NSApplication sharedApplication]; [app setActivationPolicy:NSApplicationActivationPolicyRegular];
    AppDelegate*del=[AppDelegate new]; app.delegate=del;
    NSRect fr=NSMakeRect(0,0,1140,860);
    NSWindow*win=[[NSWindow alloc] initWithContentRect:fr styleMask:NSWindowStyleMaskTitled|NSWindowStyleMaskClosable|NSWindowStyleMaskResizable backing:NSBackingStoreBuffered defer:NO];
    [win setTitle:@"Dino-Lite (Metal) — SXGA"]; [win center];
    NSView*cv=win.contentView;
    MTKView*mv=[[MTKView alloc] initWithFrame:NSMakeRect(10,70,1120,780) device:MTLCreateSystemDefaultDevice()];
    mv.colorPixelFormat=MTLPixelFormatBGRA8Unorm; mv.preferredFramesPerSecond=30;
    mv.autoresizingMask=NSViewWidthSizable|NSViewHeightSizable; [cv addSubview:mv];
    Renderer*r=[[Renderer alloc] initWithView:mv]; mv.delegate=r;
    Btns*b=[Btns new]; b.r=r;

    NSButton*cap=[[NSButton alloc] initWithFrame:NSMakeRect(10,20,140,30)]; cap.title=@"Capture (PNG)"; cap.bezelStyle=NSBezelStyleRounded; cap.target=b; cap.action=@selector(cap:); [cv addSubview:cap];
    NSTextField*ll=[NSTextField labelWithString:@"LED:"]; ll.frame=NSMakeRect(170,25,40,20); [cv addSubview:ll];
    NSSegmentedControl*seg=[[NSSegmentedControl alloc] initWithFrame:NSMakeRect(210,18,120,30)]; seg.segmentCount=2;
    [seg setLabel:@"Off" forSegment:0];[seg setLabel:@"On" forSegment:1];
    seg.selectedSegment=1; seg.target=b; seg.action=@selector(led:); [cv addSubview:seg];
    NSButton*gchk=[NSButton checkboxWithTitle:@"Gray" target:b action:@selector(gray:)]; gchk.frame=NSMakeRect(460,22,62,22); [cv addSubview:gchk];
    NSButton*awb=[[NSButton alloc] initWithFrame:NSMakeRect(525,20,80,30)]; awb.title=@"Lock WB"; awb.bezelStyle=NSBezelStyleRounded; awb.target=b; awb.action=@selector(awb:); [cv addSubview:awb];
    NSButton*ph=[[NSButton alloc] initWithFrame:NSMakeRect(610,20,120,30)]; ph.title=@"Bayer: BGGR"; ph.bezelStyle=NSBezelStyleRounded; ph.target=b; ph.action=@selector(phase:); [cv addSubview:ph];
    NSTextField*cl=[NSTextField labelWithString:@"Contrast"]; cl.frame=NSMakeRect(740,25,60,20); [cv addSubview:cl];
    NSSlider*cs=[NSSlider sliderWithValue:1.8 minValue:0.6 maxValue:3.0 target:b action:@selector(contrast:)]; cs.frame=NSMakeRect(800,22,150,22); [cv addSubview:cs];
    NSTextField*st=[NSTextField labelWithString:@"hw AE · Space = capture"]; st.frame=NSMakeRect(960,25,170,20); [cv addSubview:st]; r.status=st;

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
