#include<stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <math.h> 
#include <time.h>

// BMP file header and information header structure definitions
#pragma pack(push, 1)  // Ensure structures do not have extra bytes due to alignment
typedef struct {
    unsigned short type;           // File type, must be 0x4D42
    unsigned int size;             // File size
    unsigned short reserved1;      // Reserved
    unsigned short reserved2;      // Reserved
    unsigned int offset;           // Offset from file header to actual bitmap data
} BITMAPFILEHEADER;

typedef struct {
    unsigned int size;             // Size of the information header
    int width;                     // Image width
    int height;                    // Image height
    unsigned short planes;         // Number of planes, must be 1
    unsigned short bitCount;       // Bits per pixel
    unsigned int compression;      // Compression type
    unsigned int sizeImage;        // Image size in bytes
    int xPelsPerMeter;             // Horizontal resolution
    int yPelsPerMeter;             // Vertical resolution
    unsigned int clrUsed;          // Number of colors actually used in the bitmap
    unsigned int clrImportant;     // Number of important colors used in the display process
} BITMAPINFOHEADER;
#pragma pack(pop)


#define IMAGE_SIZE 28
#define POOLED_SIZE 13
#define FILTER_SIZE 3
#define NUM_CLASSES 10
#define NUM_FILTERS 3

///////////////////////////////////////
/// 640x480 version!
/// test VGA with hardware video input copy to VGA
// compile with
// gcc test.c -o fp1 -lm -g
// scp C:\Users\jy874\Desktop\video_sram_master_sram_vga\test.c root@10.253.17.20:/home/root/yjc_final
///////////////////////////////////////

typedef signed short fix;

#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_ONCHIP_BASE_0      0xC9000000
#define FPGA_ONCHIP_BASE_1      0xCa000000
#define FPGA_ONCHIP_SPAN      0x00080000
#define HW_REGS_SPAN          0x00005000
#define HW_REGS_BASE          0xff200000

#define FPGA_ONCHIP_END       0xC803FFFF
#define FPGA_ONCHIP_END_0       0xC903FFFF
#define FPGA_ONCHIP_END_1       0xCa03FFFF

#define START_OFFSET 0x00
#define LOAD_SIGN_OFFET 0x10
#define FC_SIGN_OFFSET 0x30

#define float2fix(a) ((fix)(a * 8192.0))
#define fix2float(a) ((float)(a) / 8192.0)

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_disc (int, int, int, short);
int  VGA_read_pixel(int, int) ;
int  video_in_read_pixel(int, int);
void draw_delay(void) ;


// the light weight buss base
void *h2p_lw_virtual_base;

// RAM FPGA command buffer
volatile unsigned short * sram_ptr = NULL ;
void *sram_virtual_base;

// RAM FPGA command buffer
volatile unsigned short * sram_ptr_0 = NULL ;
void *sram_virtual_base_0;

// RAM FPGA command buffer
volatile unsigned short * sram_ptr_1 = NULL ;
void *sram_virtual_base_1;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

volatile unsigned int *zj = NULL;
volatile unsigned int *load_sign = NULL;
volatile unsigned int *fc_sign = NULL;

// /dev/mem file id
int fd;

// pixel macro
// !!!PACKED VGA MEMORY!!!
#define VGA_PIXEL(x,y,color) do{\
	char  *pixel_ptr ;\
	pixel_ptr = (char *)vga_pixel_ptr + ((y)*640) + (x) ;\
	*(char *)pixel_ptr = (color);\
} while(0)
	

// measure time
struct timeval t1, t2;
double elapsedTime;
struct timespec delay_time ;

void normalize_image(unsigned char *input, float *output, int rows, int cols, float mean, float std_dev) {
    int i, j;
    int index;
    float scale = 1.0 / 255.0;  // Scaling factor to convert pixels to [0, 1]

    // Normalize image data
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            index = i * cols + j;
            // Scale pixel to [0, 1], then normalize using the given mean and std_dev
            output[index] = (input[index] * scale - mean) / std_dev;
        }
    }

    // Optionally, print the normalized data
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            //printf("%f ", output[i * cols + j]);
        }
        //printf("\n");
    }
    printf("normalized image loaded.\n");
}


int read_csv(const char *filename, float * weights, int nrow, int ncol) {
      FILE *fp;
      float data[nrow][ncol];

      fp = fopen(filename, "r");
      if (fp == NULL) {
            printf("Error: Unable to open file.\n");
            return 1;
      }

      // Read and store data
      int i,j;
      for ( i = 0; i < nrow; i++) {
            for ( j = 0; j < ncol; j++) {
                  if (fscanf(fp, "%f,", &weights[i*ncol+j]) != 1) {
                        printf("Error: Failed to read data from file.\n");
                        return 1;
                  }
            }
      }

      // print the data 
      for ( i = 0; i < nrow; i++) {
            for ( j = 0; j < ncol; j++) {
                  //printf("%f ", weights[i*ncol+j]);
            }
            printf("\n\n");
      }

      fclose(fp);

      return 0;
}

// read the image
void loadBMP(const char *filename, float *image_converted, int nrow, int ncol) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Unable to open file!\n");
        return;
    }

    BITMAPFILEHEADER bmpFileHeader;
    BITMAPINFOHEADER bmpInfoHeader;

    // 读取文件头
    fread(&bmpFileHeader, sizeof(BITMAPFILEHEADER), 1, file);
    fread(&bmpInfoHeader, sizeof(BITMAPINFOHEADER), 1, file);

    if (bmpFileHeader.type != 0x4D42) {
        printf("Error: Not a BMP file!\n");
        fclose(file);
        return;
    }

    int width = bmpInfoHeader.width;
    int height = bmpInfoHeader.height;
    unsigned char *image = malloc(width * height * sizeof(unsigned char));  // Allocate array of grayscale values
    unsigned char *image_crop = malloc(width * height * sizeof(unsigned char));  // Allocate array of grayscale values
    // float *image_converted = malloc(width * height * sizeof(float));  // Allocate array of grayscale values

    fseek(file, bmpFileHeader.offset, SEEK_SET);  // Move to the beginning of the data

    // Read image data
    fread(image, sizeof(unsigned char), width * height, file);

    // Print image data
    int i,j,k;
    int index, index2;
    for (i = height-1, k = 0; i >= 0; i--, k++) {
        for (j = 0; j < width; j++) {
            index = i * width + j;
            index2 = k * width + j;
            printf("%5d", *(image+index));
            image_crop[index2] = *(image+index);
        }
        printf("\n");
    }
    printf("----------------------------------------------------------\n");

    normalize_image(image_crop, image_converted, nrow, ncol, 0.5, 0.5);

    free(image);
    fclose(file);
}



int main(){
    float input_image[IMAGE_SIZE * IMAGE_SIZE];  
    float feature_maps[NUM_FILTERS][26][26];
    float feature_maps_relu[NUM_FILTERS][26][26];

    float pooled_output[NUM_FILTERS][POOLED_SIZE][POOLED_SIZE];
    float flat_output[NUM_FILTERS * POOLED_SIZE * POOLED_SIZE];

    float weights_conv_fp[NUM_FILTERS * FILTER_SIZE * FILTER_SIZE]; 
    float weights_fc_fp[NUM_FILTERS * POOLED_SIZE * POOLED_SIZE * NUM_CLASSES]; 

    float output_fc[NUM_CLASSES]; 

    loadBMP("./test.bmp", input_image, 28, 28);
    read_csv("./fc1_weights.csv", weights_fc_fp, 10, 507);
    read_csv("./conv1_weight.csv", weights_conv_fp, 9, 3);

	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 
  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	// for light weight bus
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}

	// === get RAM FPGA parameter addr =========
	sram_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE); //fp	
	sram_virtual_base_0 = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE_0); //fp	
	sram_virtual_base_1 = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE_1); //fp	
	
	if( sram_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    // Get the address that maps to the RAM buffer
	sram_ptr =(unsigned short *)(sram_virtual_base);
	sram_ptr_0 =(unsigned short *)(sram_virtual_base_0);
	sram_ptr_1 =(unsigned short *)(sram_virtual_base_1);

	load_sign = (signed int *)(h2p_lw_virtual_base + LOAD_SIGN_OFFET);
    fc_sign = (signed int *)(h2p_lw_virtual_base + FC_SIGN_OFFSET);

    
	*fc_sign = 0;
	int yjc = 0;
	int exit = 0;
    int count = 0;

    fix input_image_fpga[28*28]; 
    fix weights_fc_fpga[507*10]; 
    fix weights_conv_fpga[27]; 

    for(yjc=0; yjc < 28*28; yjc++){
        input_image_fpga[yjc] = float2fix(input_image[yjc]);
    }

    for(yjc=0; yjc< 507*10; yjc++){
        weights_fc_fpga[yjc] = float2fix(weights_fc_fp[yjc]);
    }

    for(yjc=0; yjc<27; yjc++){
        weights_conv_fpga[yjc] = float2fix(weights_conv_fp[yjc]);
    }
    printf("conversion_done");
	while(exit != 2535){
        if (count == 0){
            for(yjc = 0;yjc<28*28;yjc++){
                //*(sram_ptr + i) = rand();
                *(sram_ptr_0 + yjc) = input_image_fpga[yjc];
                
            }
            for (yjc = 0; yjc<27;yjc++){
                *(sram_ptr_1 + yjc) = weights_conv_fpga[yjc];
            }
            count = 1;
            printf("conv_done");
        }
       
        if (*load_sign == 1){
			for(yjc = 0;yjc<2535;yjc++){
				*(sram_ptr + yjc) = weights_fc_fpga[yjc+2535];
				
				*(sram_ptr_1 + yjc) = weights_fc_fpga[yjc];
				exit++;
                if(exit==2535){
                    *fc_sign = 1;
                }
			}
		}
	}
	
}
