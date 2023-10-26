import std/[strutils, strformat, sequtils]
import serial

const modBusVer* = "0.4.0"

type
  Modbus = ref object
    port: string
    baud: int32
    parity: Parity
    numBites: byte
    stopBits: StopBits
    dataByteFinal: seq[byte]
    crcFail: bool = false
    serialFail: bool = false
    txMode: bool = false

proc newModbusUart*(mport: string; mbaud: int32 = 9600; mparity: string = "none"; mnumbites: byte = 8; mstopbites: uint8 = 2): Modbus =
  ## Procedure to initialize the serial bus
  ##
  ## **Parameters:**
  ## - *mport* = Name of the serial port to use (for example "/dev/ttyUSB0" or "COM2"): string
  ## - *mbaud* = Speed to use on the serial: int32
  ## - *mparity* = Parity mathon in use ("none" "even" "odd"): string
  ## - *mnumbites* = Number of bits used for the transmission of the data (9600): byte
  ## - *mstopbites* = Number of bits used for the end transmission (1): uint8 ]
  var
    temParity: Parity
    temBitStop: StopBits
  case mparity #scegli la parità da usare
  of "none": temParity = Parity.None #nessuna parità (default).
  of "even": temParity = Parity.Even #parità pari.
  of "odd": temParity = Parity.Odd #parità dispari.
  else: temParity = Parity.None
  case mstopbites #scegli il numero di bit di stop
  of 1: temBitStop = StopBits.One #usa un pit di stop.
  of 2: temBitStop = StopBits.Two #usa 2 bit di stop (default).
  else: temBitStop = StopBits.One
  
  result = Modbus(port: mport, baud: mbaud, parity: temParity, numbites: mnumbites, stopBits: temBitStop, txMode: false)

proc crcModbus(dati: openArray[byte]): uint16 =
  const
    polinomio: uint16 = 0xA001 #polinomio divisore.
  var
    crc: uint16 = 0xFFFF #carico crc con il valore 0xFFFF.
  for bytes in dati: #controllo di tutti i bytes dal dato da inviare...
    crc = crc xor bytes #faccio xor con crc ed il primo byte.
    for bits in 0..7: #devo controllare tutti i bit del dato (byte) in esame.
      if (crc and 0x001) != 0: #controllo se il bit LSB è zero o uno, se 1...
        crc = crc shr 1 #sposto a dx il registro crc di 1 sola posizione...
        crc = crc xor polinomio # ..e faccio xor tra crc ed il divisore (polinomio MODBUS).
      else: #se invece è zero...
        crc = crc shr 1 # sposto solo di un bit a dx il registro crc senza fare xor.
  result = crc
  
proc byteLow(data: uint16): byte = #splitta il numero 16bit in un bite (parte bassa).
  result = byte(data and 0xFF) 
  
proc byteHIGH(data: uint16): byte = #splitta il numero 16bit in un bite (parte alda).
  result = byte(data shr 8)

proc clearBuffer(self: Modbus; data: seq[byte]; index: int32) = #pulsce i dati per tornare solo i dati senza crc ed indirzzi.
  self.dataByteFinal = data[3..index] #tiene solo i valori dei registri e elimina testa e coda.

proc modBusCom(self: Modbus; data: array[8, byte]) = #proc per la trasmissione del dato in MODBUS:
  var
    bufferInHead: array[0..2, byte] #crea un arri fiisso per i dati di testa (son sempre 3 Byte)
    bufferDataFinal: seq[byte] #crea uan sequanza (variabile) ci potrebbero esseree piu dati non si sa a priori.
    index: int32
  try: #prova se la seriale  è collegta se si....
    let port = newSerialPort(self.port) #crea oggetto porta seriale.
    port.open(self.baud, self.parity, self.numBites, self.stopBits, readTimeout = 500) #inizializza i parametri correti di tx.
    discard port.write(addr data[0], int32(len(data))) #scrive sulla porta il dato completo (con crc).
    discard port.read(addr bufferInHead[0], 3) #leggi i primi 3 Byte arrivati sulla seriale.
    index = int32(bufferInHead[2]*2) # modb dice quanti dati registri ci sono, ma vanno moltiplicati x 2 (1 = 2 da 8bit).
    bufferDataFinal.setLen(index) #inizializza la sequenza con il numero di Byte in arrivo.
    discard port.read(addr bufferDataFinal[0], index) #legge seriale e metti i dati a partire dal primo elemento di bufferDataFinal.
    bufferDataFinal = bufferInHead.toSeq() & bufferDataFinal #unisci "test" e "coda" per fare il crc di ricezione (se = 0 è ok).
    port.close() #chiudi la seriale.
  except InvalidSerialPortError:
    self.serialFail = true
  if crcModbus(bufferDataFinal) == 0: #se il crc ricevuto = 0 allora nessun errore di trasmissione.
    self.crcFail = false #variabile posta a false..gestibile nel main.
    self.clearBuffer(bufferDataFinal, index)
  else: #se 1= 0 allora ce stato un problema.
    self.crcFail = true #poni la variabile a true.. puoi gestire nel prog principale l'errore.

proc showRegister*(self: Modbus): seq[byte] = #proc per richiedere un valore memorizato senza interrogare il dispositivo.
  ## Show the stored values, without requesting the device on serial.
  result = self.dataByteFinal
  
proc readRegister(self: Modbus; addrDevice, addrRegister, numrRegRead: byte, typeReg: byte): seq[byte] = #proc creazione byte da trasmettere.
  var byteTx: array[0..7, byte] 
  if self.txMode == false: #se siamo in modalità UART...
    byteTx[0] = addrDevice #1- byte metti l'indirizzo fisico del dispositivo.
  byteTx[1] = typeReg #2- scrive il tipo di registro da interrogare.
  byteTx[2] = byteHIGH(addrRegister) #3- scrive il byte ALTO dell'indirizzo del registro da interrogare.
  byteTx[3] = byteLOW(addrRegister) #4- scrive il byte BASSO dell'indirizzo del registro da interrogare.
  byteTx[4] = byteHIGH(numrRegRead) #5- scrive il byte ALTO den numero di registri consecutivi da interrogare.
  byteTx[5] = byteLow(numrRegRead) #6- scrive il byte BASSO den numero di registri consecutivi da interrogare.
  if self.txMode == false: #se siamo in modalità UART...
    let crc = crcModbus(byteTx[0..5]) #chiama la funzione di conctrollo cec e fallo con i prim 6 Byte.
    byteTx[6] = byteLOW(crc) #scrive il Byte BASSO del crc ricavato (ATTENZIONE!! il byte scritto inverso ecco perhè prima Byte LOW).
    byteTx[7] = byteHIGH(crc) #scrive il Byte ALTO del crc ricavato (ATTENZIONE!! il byte scritto inverso ecco perhè prima Byte LOW).
  self.modBusCom(byteTx) #solo ra manda tutto alla procedura di tTRASMISSIONE dati. -->
  result = self.dataByteFinal #ritorna indietro il risultato finale (pulito).

proc readInputRegister*(self: Modbus; addrDevice, addrRegister, numrRegRead: byte): seq[byte] = #proc alias pr INPUT Register
  ## This function code is used to read from 1 to 125 contiguous input registers in a remote device 0x04.
  ##
  ## **Parameters:**
  ## - *addrDevice* = Address of the connected physical device.
  ## - *addrRegister* = Address of the register to be examined.
  ## - *numrRegRead* = Number of **consecutive** registers to read.
  result = self.readRegister(addrDevice, addrRegister, numrRegRead, typeReg = 0x04) #chiama la fera proc di generazione con 0x04.
  
proc readHoldingRegister*(self: Modbus; addrDevice, addrRegister, numrRegRead: byte):seq[byte]= #proc alias pr HOLDINGRegister
  ## This function code is used to read from 1 to 125 contiguous input registers in a remote device 0x03.
  ##
  ## **Parameters:**
  ## - *addrDevice* = Address of the connected physical device.
  ## - *addrRegister* = Address of the register to be examined.
  ## - *numrRegRead* = Number of **consecutive** registers to read.
  result = self.readRegister(addrDevice, addrRegister, numrRegRead, typeReg =0x03) #chiama la fera proc di generazione con 0x03

proc isCrcFail*(self: Modbus): bool =
  ## Return "true"if CRC control has been successful.
  ## Return "false" if CRC control is not correct.
  result = self.crcFail

proc isSerialFail*(self: Modbus): bool =
  ## Return "true"if Serial connection has been successful.
  ## Return "false" if Serial connection is failed.
  result = self.serialFail
  
when isMainModule:
  let
    datoTest: array[0..5, byte] = [0x02, 0x04, 0x00, 0x56, 0x00, 0x01] #crc= 209, 233
  
  let gefran = newModbusUart("/dev/ttyUSB0", 9600, "none", 8, 2)
  var  test123 = gefran.readInputRegister(2, 86, 1)
  if gefran.isCrcFail() == false:
    echo(fmt"Temperatura: {test123[0]+test123[1]}°C")
  else:
    echo"CRCFALLITO!!!!!!!!!!!"

