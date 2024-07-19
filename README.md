# HFM Adapter

## With this adapter it is possible to replace any HLM with any HFM.

## Development status


- [Hardware: Revision 1.0 done](./Rev 1.0/README.md)
- Software: Version 1.0 done

### HFM Adapter based on

- STM32 G431KB Nucleo-32 Board
- ST M24M02 2MBit serial I²C EEPROM to store the tables and configuration
- TI REF02 +5.0v precision voltage reference, +-0.2%
- Infineon TLE4252D tracking regulator 250mA
- TI OPA2992 output operation amplifier

### Features

- Automotive grade components
- 4 layer PCB
- Configuration via TunerStudio

### Inputs

- two HFM buffered analog inputs (0v - 5v) with overvoltage protection 
- USB for configuration
- VBat input for voltage monitoring 

### Outputs

- two analog outputs
- two precision reference 5.0v outputs for HFM
- two Vbat outputs for HFM

### Housing

- Header: Amphenol ATM13-12PA
- Enclosure: Amphenol AIPXE-325X4

### Pin terminal for BMW M70 adapter

- pin terminal 1252 1718125
- connector boot 1252 1711262
- pins
  - 0.5 - 1.0mm² 6113 1376191
  - 1.0mm² - 2.5mm² 6113 1376193
  - 2.5mm² - 4.0mm² 6113 1376195