#include "medianFilter.h"

static void Array_sort(uint16_t *dataArray , int n);
static uint16_t median(uint16_t *dataArray , int n);

//////////////////////
// Metodos publicos //
//////////////////////

bool medianFilter::create_file(void){
  bool sts;
  
  // Activamos la SD
	digitalWrite(MF_SD_PIN, LOW);

  Sd.begin(MF_SD_PIN, SPI_FULL_SPEED);
	// En caso de no existir creamos el directorio
	sts = Sd.mkdir(pathName);		// Creamos la carpeta si no existe
	sts = Sd.chdir(pathName);		// Configuramos la direccion

  // Creamos el archivo
  if (!Sd.exists(fileName)){
    sts = MyFile.open(fileName, O_WRITE | O_CREAT);
    for(uint32_t i=0; i < MF_WINDOWS_LENGTH * MF_NSAMPLES; i++){            // Iteramos para cada registro
      MyFile.write((0 >> 8) & 0xFF);      // Agregamos el byte mas significativo
      MyFile.write(0 & 0xFF);         // Agregamos el byte menos significativo
      MyFile.write(MF_LINE_TERMINATOR);           // Agregamos el separador
    }
    MyFile.close(); 
  }
  digitalWrite(MF_SD_PIN, HIGH);
  return sts;
}


void medianFilter::addData(uint16_t *data, int NSamples){  
  // Activamos la SD
  digitalWrite(MF_SD_PIN, LOW);
  digitalWrite(10, HIGH);
  Sd.begin(MF_SD_PIN, SPI_FULL_SPEED);
  Sd.chdir(pathName);   // Configuramos la direccion
  MyFile.open(fileName, O_WRITE | O_CREAT);

  MyFile.seekSet(3 * actualWire * MF_NSAMPLES);
  char buff;
  // Loop que solo itera hasta NSAmples
  
  for (int i=0; i<NSamples; i++){
    buff = (data[i] >> 8) & 0xFF;
    MyFile.write(buff);
    buff = data[i] & 0xFF;
    MyFile.write(buff);
    MyFile.write(MF_LINE_TERMINATOR);
  }
  MyFile.close();
  actualWire += 1;
  if (actualWire >= MF_WINDOWS_LENGTH) actualWire = 0;
  
  // Escribimos los datos
  digitalWrite(MF_SD_PIN, HIGH);
}


void medianFilter::getFilterData(uint16_t *data, int NSamples){
  uint16_t dataWindow[MF_WINDOWS_LENGTH];
  // Activamos la SD
  digitalWrite(MF_SD_PIN, LOW);
  digitalWrite(10, HIGH);
  Sd.begin(MF_SD_PIN, SPI_FULL_SPEED);

  Sd.chdir(pathName);   // Configuramos la direccion
  MyFile.open(fileName,  O_READ | O_CREAT);

  for(int i=0; i<NSamples; i++){
    for(int j=0; j<MF_WINDOWS_LENGTH; j++) {
      dataWindow[j] = readValue(i, j);
    }
    data[i] = median(dataWindow, MF_WINDOWS_LENGTH);
  }
  MyFile.close();
  digitalWrite(MF_SD_PIN, HIGH);
}



uint16_t medianFilter::readValue(int idx, int n_wire){
  uint32_t pos = 3*n_wire*MF_NSAMPLES + 3*idx;
  MyFile.seekSet(pos);
  uint8_t buff[2];
  uint16_t value;
  buff[0] = MyFile.read();
  buff[1] = MyFile.read();
  value = (buff[0] << 8) + buff[1];
  return value;
}

static uint16_t median(uint16_t dataArray[], int n){
  Array_sort(dataArray, n);
  return dataArray[n/2];
}


static void Array_sort(uint16_t *dataArray , int n)
{ 
  // declare some local variables
  int i=0 , j=0 , temp=0;

  for(i=0 ; i<n ; i++)
  {
    for(j=0 ; j<n-1 ; j++)
    {
      if(dataArray[j]>dataArray[j+1])
      {
        temp        = dataArray[j];
        dataArray[j]    = dataArray[j+1];
        dataArray[j+1]  = temp;
      }
    }
  }
}
