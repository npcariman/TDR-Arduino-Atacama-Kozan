#ifndef _MEDIAN_FILTER_
#define _MEDIAN_FILTER_

// Incluimos las librerias a utilizar
#include "Arduino.h"
#include "SdFat.h"
#include "SPI.h"

// Instanciamos las clases que usamos
extern SdFat Sd;
extern SdFile MyFile;

// Definimos los macros a utilizar
#define MF_SD_PIN           4
#define MF_SPI_SPEED        SPI_FULL_SPEED
#define MF_WINDOWS_LENGTH   5
#define MF_DELIMITER        ';'
#define MF_LINE_TERMINATOR  '\n'
#define MF_NSAMPLES         700



class medianFilter{
public:
    uint8_t windows_length = MF_WINDOWS_LENGTH;
    uint8_t actualWire = 0;
    char fileName [50] = "median_filter.csv"; 
    char pathName   [50] = "/.median_filter";

    bool create_file(void);
    void addData(uint16_t *data, int NSamples);
    void getFilterData(uint16_t *data, int NSamples);
private:
    uint8_t idx = 0;    
    uint16_t readValue(int idx, int n_wire);
};


#endif
