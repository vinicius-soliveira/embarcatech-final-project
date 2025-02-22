#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "ssd1306/ssd1306.h"
#include "neopixel.c"

#define MICROPHONE_PIN          28
#define MICROPHONE_CHANNEL      2
#define NPLED_PIN               7
#define NPLED_COUNT             25
#define I2C_SDA_PIN             14
#define I2C_SCL_PIN             15
#define BTN_A_PIN               5
#define DEBOUNCE_DELAY_MS       50

// Parâmetros e macros do ADC.
#define ADC_CLOCK_DIV           12000.f //48M/(12000) = 4kSps
#define SAMPLES                 4096   // Número de amostras que serão feitas do ADC.
#define ADC_ADJUST(x)           (x * 3.3f / (1 << 12u) - 1.65f) // Ajuste do valor do ADC para Volts.

// Frequências para as notas da guitarra (em Hz)
#define FREQ_E2                 82.41
#define FREQ_A2                 110.00
#define FREQ_D3                 146.83
#define FREQ_G3                 196.00
#define FREQ_B3                 246.94
#define FREQ_E4                 329.63

#define FREQ_RESOLUTION         (float)(4000.0/4096.0)
#define PI                      3.14159265358979323846
#define NUM_OF_STRINGS          6
#define DELTA_FREQ_FOR_TUNNING  0.1225 // Desvio relativo entre um tom, e os tons abaixo ou acima
#define MAX_DEV_TO_BE_TUNED     0.5 // Margem para considerar afinado: 0,5 Hz abaixo ou acima


// Enum para os estados da máquina
typedef enum {
    IDLE,
    DETECTING,
    TUNING,
    LOW,
    HIGH,
    CORRECT
} state;

// Enum notas da guitarra em afinação padrão
enum note {E2, A2, D3, G3, B3, E4, UNKNOWN};
 
//Enum para condição de afinação da corda
enum string_tune {VERY_DOWN, DOWN, TUNED, UP, VERY_UP};

//Estrutura para representar uma corda da guitara
typedef struct {
    int note;
    float desired_freq;
} string_t;

// Estrutura para representar números complexos
typedef struct {
    float r;
    float i;
} complex_t;

// Preparar área de renderização para o display
struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

// Canal e configurações do DMA
uint dma_channel;
dma_channel_config dma_cfg;

// Buffer de amostras do ADC.
uint16_t adc_buffer[SAMPLES];
complex_t fft_data[SAMPLES];
void sample_mic();
void fft_pre_calc(uint16_t*, int, complex_t*);
void hann_window(complex_t*, int);
void fft(complex_t*, int);
float magnitude(complex_t);
float find_peak(complex_t*, int);
bool check_note(float);


uint8_t oled[ssd1306_buffer_length];
void print_on_oled(uint8_t*, char**, int, int, int);

void oled_config(){
     // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);
    memset(oled, 0, ssd1306_buffer_length);
    render_on_display(oled, &frame_area);
}

void adc_dma_config(){
    adc_gpio_init(MICROPHONE_PIN);
    adc_init();
    adc_select_input(MICROPHONE_CHANNEL);

    adc_fifo_setup(
        true,   // Habilitar FIFO
        true,   // Habilitar request de dados do DMA
        1,      // Threshold para ativar request DMA é 1 leitura do ADC
        false,  // Não usar bit de erro
        false   // Não fazer downscale das amostras para 8-bits, manter 12-bits.
    );

    adc_set_clkdiv(ADC_CLOCK_DIV);

    dma_channel = dma_claim_unused_channel(true);

    // Configurações do DMA.
    dma_cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); // Tamanho da transferência é 16-bits (usamos uint16_t para armazenar valores do ADC)
    channel_config_set_read_increment(&dma_cfg, false);            // Desabilita incremento do ponteiro de leitura (lemos de um único registrador)
    channel_config_set_write_increment(&dma_cfg, true);            // Habilita incremento do ponteiro de escrita (escrevemos em um array/buffer)

    channel_config_set_dreq(&dma_cfg, DREQ_ADC);                   // Usamos a requisição de dados do ADC

}

void set_LED_matrix(int);

volatile bool button_pressed = false;
static void gpio_callback(uint gpio, uint32_t events);

char *note_labels[] = {"E2", "A2", "D3", "G3", "B3", "E4"};

string_t strings[6] = {
    {E2, FREQ_E2}, // E2
    {A2, FREQ_A2}, // A2
    {D3, FREQ_D3}, // D3
    {G3, FREQ_G3}, // G3
    {B3, FREQ_B3}, // B3
    {E4, FREQ_E4}  // E4
};

string_t current_string = {UNKNOWN, 0.0f};

static const unsigned char image_bitmap[] = {
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0x00,0xff,0xff,
    0xff,0xff,0xff,0xff,0x3f,0x21,0xfe,0xff,0xff,0xff,0xff,0xff,0x1f,0x08,0xfe,
    0xff,0xff,0xff,0xff,0xff,0x0f,0x02,0xf8,0xff,0xff,0xff,0xff,0xff,0x4f,0x60,
    0xfc,0xff,0xff,0xff,0xff,0xff,0x07,0xf8,0xf1,0xff,0xff,0xff,0xff,0xff,0x23,
    0xfe,0xf9,0xff,0xff,0xff,0xff,0xff,0x0b,0xfe,0xf3,0xff,0xff,0xff,0xff,0xff,
    0x81,0xff,0xe7,0xef,0xff,0xff,0xff,0xff,0x01,0xdf,0xe7,0x87,0xff,0xff,0xff,
    0xff,0xc8,0x87,0xef,0x87,0xff,0xff,0xff,0xff,0xc0,0x83,0xef,0x03,0xff,0xff,
    0xff,0x7e,0xc2,0x11,0xcf,0x23,0xff,0xff,0x0f,0x40,0xf0,0x05,0xef,0x03,0xfe,
    0xff,0x07,0x00,0xe0,0xc1,0xdf,0x13,0xff,0xff,0x81,0x02,0xf1,0xe0,0xcf,0x03,
    0xfe,0xff,0xe1,0x3f,0x80,0xf0,0xdf,0x43,0xfe,0xff,0xe8,0x3f,0x04,0xf0,0xdf,
    0x87,0xfe,0xff,0xf0,0x1f,0x01,0xf2,0xcf,0x47,0xfe,0x7f,0xf8,0x1f,0x94,0xc0,
    0xdf,0xcf,0xfe,0x7f,0xf8,0x0f,0x3e,0x00,0xdf,0x7f,0xfe,0x7f,0xfa,0x07,0x7f,
    0x04,0xcc,0xff,0xfe,0x7f,0xf8,0x47,0x7f,0x48,0xc0,0x7f,0xfe,0x7f,0xfc,0x83,
    0x7f,0x31,0xc0,0x7f,0xfe,0x7f,0xf8,0x91,0x7f,0xf8,0xc9,0x3f,0xff,0x7f,0xf8,
    0xc0,0x7f,0xf0,0x07,0x1f,0xff,0x7f,0x22,0xe4,0x7f,0xf2,0x0f,0x80,0xff,0xff,
    0x00,0xf0,0xff,0xf0,0x27,0x80,0xff,0xff,0x00,0xf9,0x7f,0xe0,0xe3,0xd1,0xff,
    0xff,0x11,0xfc,0xff,0x80,0xe0,0xef,0xff,0xff,0x43,0xff,0xff,0x08,0xf2,0xff,
    0xff,0xff,0xff,0xff,0xff,0x80,0xe0,0xff,0xff,0xff,0xff,0xff,0xff,0x04,0xf0,
    0xff,0xff,0xff,0xff,0xff,0xff,0x21,0xf8,0xff,0xff,0xff,0xff,0xff,0xff,0x03,
    0xf9,0xff,0xff,0xff,0xff,0xff,0xff,0x03,0xfc,0xff,0xff,0xff,0xff,0xff,0xff,
    0x4f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xbf,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff
};

char message[8][16] = {NULL};
float detected_freq = 0.0;
float dev_freq; 

int main() {
    stdio_init_all();
    
    /*
    //Configuração para uso de bitmap
        ssd1306_t ssd_bm;
        ssd1306_init_bm(&ssd_bm, 128, 64, false, 0x3C, i2c1);
        ssd1306_config(&ssd_bm);
    */ 

    adc_dma_config();
    npInit(NPLED_PIN, NPLED_COUNT);

    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);

    state current_state = IDLE;
    int i = -2;
    bool up_down = 1;
   
    //ssd1306_draw_bitmap(&ssd_bm, image_bitmap);
    //sleep_ms(3000);

    oled_config();

    memset(oled, 0, ssd1306_buffer_length);
    render_on_display(oled, &frame_area);

    while (1) {
        if (up_down) {
            i = (i + 1) % 25;
            up_down = i == 24 ? false : true;
        } else {
            i = (i - 1) % 25;
            up_down = i == 0 ? true : false;
        }
        npClear();
        switch (current_state) {
            case IDLE:
                memset(message, 0, sizeof(message));
                    message[0] = "  Embarcatech   ";
                    message[1] = "                ";
                    message[2] = "    Afinador    ";
                    message[3] = "    Digital     ";
                print_on_oled(oled, message, 4, 5, 0);
                sleep_ms(2000);
                current_state = DETECTING;
                break;

            case DETECTING:
                memset(message, 0, sizeof(message));
                message[0] = "    Captando   ";
                message[1] = "    o som da   ";
                message[2] = "    guitarra   ";
                              
                print_on_oled(oled, message, 4, 6, 8);
                
                npSetLED(i,0, 0, 128);
                npWrite();
                
                sample_mic();
                fft_pre_calc(adc_buffer, SAMPLES, fft_data);
                hann_window(fft_data, SAMPLES);
                fft(fft_data, SAMPLES);
                detected_freq = find_peak(fft_data, SAMPLES);
                printf("%f\n", detected_freq);
                current_state = check_note(detected_freq) ? TUNING : DETECTING;
                break;

            case TUNING:
                dev_freq = detected_freq - current_string.desired_freq;
                if (fabs(dev_freq) < MAX_DEV_TO_BE_TUNED){
                    current_state = CORRECT;
                    set_LED_matrix(TUNED);
                }else if(dev_freq > 0.5){
                    current_state = HIGH;
                    if (dev_freq/current_string.desired_freq > DELTA_FREQ_FOR_TUNNING/2)
                        set_LED_matrix(VERY_UP);
                    else
                        set_LED_matrix(UP);
                }else if(dev_freq < -0.5){
                    current_state = LOW;
                    if (dev_freq/current_string.desired_freq < -DELTA_FREQ_FOR_TUNNING/2)
                        set_LED_matrix(VERY_DOWN);
                    else
                        set_LED_matrix(DOWN);
                }
                break;
            case LOW:
                memset(message, 0, sizeof(message));
                strcpy(message[0], "    Nota:   ");
                strcat(message[0], note_labels[current_string.note]);
                message[1] = "               ";
                message[2] = "    Aperte a   ";
                message[3] = "     corda     ";
                print_on_oled(oled, message, 4, 6, 8);

                sample_mic();
                fft_pre_calc(adc_buffer, SAMPLES, fft_data);
                hann_window(fft_data, SAMPLES);
                fft(fft_data, SAMPLES);
                detected_freq = find_peak(fft_data, SAMPLES);
                printf("%f\n", detected_freq);
                check_note(detected_freq);
                current_state = check_note(detected_freq) ? TUNING : DETECTING;
                if (current_state == DETECTING)
                    i = -1;
                break;

            case HIGH:
                memset(message, 0, sizeof(message));
                strcpy(message[0], "    Nota:   ");
                strcat(message[0], note_labels[current_string.note]);
                printf("%s\n",message[0]);
                message[1] = "               ";
                message[2] = "    Afrouxe a  ";
                message[3] = "      corda    ";
                print_on_oled(oled, message, 4, 6, 8);

                sample_mic();
                fft_pre_calc(adc_buffer, SAMPLES, fft_data);
                hann_window(fft_data, SAMPLES);
                fft(fft_data, SAMPLES);
                detected_freq = find_peak(fft_data, SAMPLES);
                printf("%f\n", detected_freq);
                check_note(detected_freq);
                current_state = check_note(detected_freq) ? TUNING : DETECTING;
                if (current_state == DETECTING)
                    i = -1;

                break;

            case CORRECT:
                message[0] = "    Nota:   ";
                message[1] = "               ";
                message[2] = "     Corda     ";
                message[3] = "    Afinada!    ";
                print_on_oled(oled, message, 4, 6, 8);
                current_state = DETECTING;  // Espera por outro sinal de áudio
                i = - 1;
                sleep_ms(1000);
                break;
        }
       
        if(button_pressed)
            current_state = IDLE;
    }

    return 0;
}

void print_on_oled(uint8_t* oled, char** message, int num_of_words, int x, int y) {
    memset(oled, 0, ssd1306_buffer_length);
    render_on_display(oled, &frame_area);

    for (uint i = 0; i < num_of_words; i++) {
        ssd1306_draw_string(oled, x, y, message[i]);
        y += 8;
    }
    render_on_display(oled, &frame_area);
}

void sample_mic() {
    adc_fifo_drain(); // Limpa o FIFO do ADC.
    adc_run(false);   // Desliga o ADC (se estiver ligado) para configurar o DMA.

    dma_channel_configure(
        dma_channel,
        &dma_cfg,
        adc_buffer,         // Escreve no buffer.
        &(adc_hw->fifo),    // Lê do ADC.
        SAMPLES,            // Faz SAMPLES amostras.
        true                // Liga o DMA.
    );

    // Liga o ADC e espera acabar a leitura.
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);

    // Acabou a leitura, desliga o ADC de novo.
    adc_run(false);
}

void fft_pre_calc(uint16_t* buffer, int size, complex_t* data){
  float conv_values[size];

  for (int i = 0; i < size; i++){
    conv_values[i] = ADC_ADJUST(buffer[i]);
    data[i].r = conv_values[i];
    data[i].i = 0;    
  }

}

void fft(complex_t *data, int size) {
    if (size <= 1) return;

    int half = size / 2;
    complex_t even[half], odd[half];
    for (int i = 0; i < half; i++) {
        even[i] = data[2 * i];
        odd[i]  = data[2 * i + 1];
    }

    fft(even, half);
    fft(odd, half);

    for (int k = 0; k < half; k++) {
        float angle = -2 * PI * k / size;
        complex_t w = { cos(angle), sin(angle) };
        complex_t t = {
            w.r * odd[k].r - w.i * odd[k].i,
            w.r * odd[k].i + w.i * odd[k].r
        };
        data[k].r = even[k].r + t.r;
        data[k].i = even[k].i + t.i;
        data[k + half].r = even[k].r - t.r;
        data[k + half].i = even[k].i - t.i;
    }
}

void hann_window(complex_t *data, int size) {
    for (int i = 0; i < size; i++) {
        float w = 0.5 * (1 - cos(2 * PI * i / (size - 1)));
        data[i].r *= w;
        data[i].i *= w;
    }
}

float magnitude(complex_t c) {
    return sqrtf(c.r * c.r + c.i * c.i);
}

float find_peak(complex_t *data, int size) {
    int peakIndex = 0;
    float maxMag = 0.0f;

    for (int i = 0; i < size/2 + 1; i++) {
        float mag = magnitude(data[i]);
        if (mag > maxMag) {
            maxMag = mag;
            peakIndex = i;
        }
    }
    return (float)(peakIndex)*FREQ_RESOLUTION;
}

bool check_note(float frequency){
  
  if (frequency < 75.0 || frequency > 350.0)
    return false;

  for (int i = E2; i <= E4; i++){
    if (fabs(frequency - strings[i].desired_freq)/strings[i].desired_freq  < DELTA_FREQ_FOR_TUNNING){
        current_string.note = i;
        current_string.desired_freq = strings[i].desired_freq;
        return true;
    }
 }
  return false;

}

// Callback da interrupção do botão
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == BTN_A_PIN) {
        static uint32_t last_interrupt_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        if (current_time - last_interrupt_time > DEBOUNCE_DELAY_MS) {
            last_interrupt_time = current_time;             
            button_pressed = true;
        }
    }
}

void set_LED_matrix(int tune){
    if (tune < VERY_DOWN || tune > VERY_UP)
        return;

    npClear();

    switch (tune){
        case VERY_DOWN:
            npSetLED(0, 128, 0, 0);
            npSetLED(1, 128, 0, 0);
            npSetLED(2, 128, 0, 0);
            npSetLED(3, 128, 0, 0);
            npSetLED(4, 128, 0, 0);
            npSetLED(5, 128, 0, 0);
            npSetLED(9, 128, 0, 0);
            npSetLED(10, 128, 0, 0);
            npSetLED(14, 128, 0, 0);
            npSetLED(15, 128, 0, 0);
            npSetLED(19, 128, 0, 0);
            npSetLED(20, 128, 0, 0);
            npSetLED(21, 128, 0, 0);
            npSetLED(22, 128, 0, 0);
            npSetLED(23, 128, 0, 0);
            npSetLED(24, 128, 0, 0);
            break;
        case DOWN:
            npSetLED(6, 128, 128, 0);
            npSetLED(7, 128, 128, 0);
            npSetLED(8, 128, 128, 0);
            npSetLED(11, 128, 128, 0);
            npSetLED(13, 128, 128, 0);
            npSetLED(16, 128, 128, 0);
            npSetLED(17, 128, 128, 0);
            npSetLED(18, 128, 128, 0);
            break;
        case TUNED:
            npSetLED(12, 0, 128, 0);
            npWrite();
            sleep_ms(250);
            npClear();

            npSetLED(6, 0, 128, 0);
            npSetLED(7, 0, 128, 0);
            npSetLED(8, 0, 128, 0);
            npSetLED(11, 0, 128, 0);
            npSetLED(13, 0, 128, 0);
            npSetLED(16, 0, 128, 0);
            npSetLED(17, 0, 128, 0);
            npSetLED(18, 0, 128, 0);

            npWrite();
            sleep_ms(250);

            npClear();

            for(int i = 0; i < NPLED_COUNT; i++){
                npSetLED(i, 0, 128, 0);
            }
            break;
        case UP:
            npSetLED(6, 128, 128, 0);
            npSetLED(7, 128, 128, 0);
            npSetLED(8, 128, 128, 0);
            npSetLED(11, 128, 128, 0);
            npSetLED(13, 128, 128, 0);
            npSetLED(16, 128, 128, 0);
            npSetLED(17, 128, 128, 0);
            npSetLED(18, 128, 128, 0);
            break;
        case VERY_UP:
            npSetLED(0, 128, 0, 0);
            npSetLED(1, 128, 0, 0);
            npSetLED(2, 128, 0, 0);
            npSetLED(3, 128, 0, 0);
            npSetLED(4, 128, 0, 0);
            npSetLED(5, 128, 0, 0);
            npSetLED(9, 128, 0, 0);
            npSetLED(10, 128, 0, 0);
            npSetLED(14, 128, 0, 0);
            npSetLED(15, 128, 0, 0);
            npSetLED(19, 128, 0, 0);
            npSetLED(20, 128, 0, 0);
            npSetLED(21, 128, 0, 0);
            npSetLED(22, 128, 0, 0);
            npSetLED(23, 128, 0, 0);
            npSetLED(24, 128, 0, 0);
    }

    npWrite();
}

