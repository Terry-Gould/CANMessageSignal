#pragma once

#include "CANMessageSignal.h"

using namespace CANMessageSignal;

// ----------------------------------------

CanMessage engine201({ .name = "Engine201",
                       .idType = STANDARD,
                       .id = 0x201,
                       .dlc = 8,
                       .defaultFill = 0xFF,
                       .comment = "From ECM, contains RPM, Vehicle Speed and APP" });

CanByteSignal rpm({ .name = "EngineRPM",
                .dataType = UNSIGNED,
                .startByte = 0,
                .byteLength = 2,
                .endianness = BIG_ENDIAN,
                .factor = 0.26,
                .offset = 0.0,
                .unit = "rpm",
                .comment = "Engine speed",
                .min = 0,
                .max = 10000,
                .defaultValue = 0,
                .overrideCapable = true });

CanByteSignal speedo({ .name = "VehicleSpeed",
                   .dataType = UNSIGNED,
                   .startByte = 4,
                   .byteLength = 2,
                   .endianness = BIG_ENDIAN,
                   .factor = 0.01,
                   .offset = -100,
                   .unit = "km/h",
                   .comment = "Vehicle speed km/h",
                   .min = 0,
                   .max = 300,
                   .defaultValue = 0,
                   .overrideCapable = true });

// -------------------------------------------

CanMessage engine240({ .name = "Engine240",
                       .idType = STANDARD,
                       .id = 0x240,
                       .dlc = 8,
                       .defaultFill = 0x00,
                       .comment = "From ECM, contains raw ECT and Engine status" });

CanByteSignal rawECT({ .name = "RawECT",
                   .dataType = UNSIGNED,
                   .startByte = 3,
                   .byteLength = 1,
                   .endianness = BIG_ENDIAN,
                   .factor = 1,
                   .offset = -40,
                   .unit = "°C",
                   .comment = "Raw ECT as calculated from the sensor, see ID 0x420 byte0 for gauge ECT",
                   .min = 0,
                   .max = 300,
                   .defaultValue = 0,
                   .overrideCapable = true });

// -------------------------------------------

CanMessage engine420({ .name = "Engine420",
                       .idType = STANDARD,
                       .id = 0x420,
                       .dlc = 7,
                       .defaultFill = 0x00,
                       .comment = "From ECM, contains ECT for dash, Odometer, Oil Pressure Gauge, Engine Light, other warning lights." });

CanByteSignal gaugeECT({ .name = "GaugeECT",
                     .dataType = UNSIGNED,
                     .startByte = 0,
                     .byteLength = 1,
                     .endianness = BIG_ENDIAN,
                     .factor = 1,
                     .offset = -40,
                     .unit = "°C",
                     .comment = "ECT send to gauge, mirrors ID 0x240.3 but defaults to 0xFF when fault/open circuit/short circuit",
                     .min = 0,
                     .max = 300,
                     .defaultValue = 0,
                     .overrideCapable = true });

EnumMap oilMap({
  { 0x00, "Low" },
  { 0x01, "Ok" },
  { 0x02, "Fault" },
});

CanSignal oilGauge({ .name = "OilGauge",
                     .dataType = UNSIGNED,
                     .startBit = 33, //byte 4
                     .bitLength = 2,
                     .endianness = BIG_ENDIAN,
                     .factor = 1,
                     .offset = 0,
                     .unit = "NA",
                     .comment = "Oil Gauge, this is not a analog value, rather just a status, oil gauge off, oil low, oil ok",
                     .min = 0,
                     .max = 3,
                     .defaultValue = 0x02,
                     .signalRole = NORMAL_SIGNAL,
                     .multiplexor = nullptr,
                     .multiplexValue = 0,
                     .enumMap = &oilMap,
                     .overrideCapable = true });

CanByteBitSignal celOn({ .name = "CheckEngineLight",
                  .byte = 5,  //byte5 bit 6
                  .bit = 6,
                  .comment = "Engine Light, bit 6. Overriden by bit 7 which is flashing",
                  .defaultValue = 0,
                  .enumMap = nullptr,
                  .overrideCapable = true });

CanBitSignal celFlashing({ .name = "FlashingEngineLight",
                        .bit = 47,  //byte5 bit 7
                        .comment = "Engine Light Flashing, bit 7. Overrides bit 6",
                        .defaultValue = 0,
                        .enumMap = nullptr,
                        .overrideCapable = true });

CanBitSignal coolantLight({ .name = "CoolantLight",
                         .bit = 49,  //byte6 bit 1
                         .comment = "Red Coolant Light",
                         .defaultValue = 0,
                         .enumMap = nullptr,
                         .overrideCapable = true });

CanSignal batteryLight({ .name = "BatteryLight",
                         .dataType = UNSIGNED,
                         .startBit = 54,  //byte6 bit 6
                         .bitLength = 1,
                         .endianness = BIG_ENDIAN,
                         .factor = 1,
                         .offset = 0,
                         .unit = "NA",
                         .comment = "Red Battery Light",
                         .min = 0,
                         .max = 1,
                         .defaultValue = 0,
                         .signalRole = NORMAL_SIGNAL,
                         .multiplexor = nullptr,
                         .multiplexValue = 0,
                         .enumMap = nullptr,
                         .overrideCapable = true });

CanSignal oilLight({ .name = "OilLight",
                     .dataType = UNSIGNED,
                     .startBit = 55,  //byte6 bit 7
                     .bitLength = 1,
                     .endianness = BIG_ENDIAN,
                     .factor = 1,
                     .offset = 0,
                     .unit = "NA",
                     .comment = "Red Oil Light",
                     .min = 0,
                     .max = 1,
                     .defaultValue = 0,
                     .signalRole = NORMAL_SIGNAL,
                     .multiplexor = nullptr,
                     .multiplexValue = 0,
                     .enumMap = nullptr,
                     .overrideCapable = true });

// ---------------------------------------------

CanMessage engine650({ .name = "Engine650",
                       .idType = STANDARD,
                       .id = 0x650,
                       .dlc = 1,
                       .defaultFill = 0x00,
                       .comment = "From ECM, contains Cruise lights" });

CanSignal cruiseLight({ .name = "CruiseLight",
                        .dataType = UNSIGNED,
                        .startBit = 6,  //byte1 bit 6
                        .bitLength = 1,
                        .endianness = BIG_ENDIAN,
                        .factor = 1,
                        .offset = 0,
                        .unit = "NA",
                        .comment = "Green CRUISE Light",
                        .min = 0,
                        .max = 1,
                        .defaultValue = 0,
                        .signalRole = NORMAL_SIGNAL,
                        .multiplexor = nullptr,
                        .multiplexValue = 0,
                        .enumMap = nullptr,
                        .overrideCapable = true });

CanSignal cruiseMainLight({ .name = "CruiseMainLight",
                            .dataType = UNSIGNED,
                            .startBit = 7,  //byte1 bit 7
                            .bitLength = 1,
                            .endianness = BIG_ENDIAN,
                            .factor = 1,
                            .offset = 0,
                            .unit = "NA",
                            .comment = "Yellow CRUISE MAIN Light",
                            .min = 0,
                            .max = 1,
                            .defaultValue = 0,
                            .signalRole = NORMAL_SIGNAL,
                            .multiplexor = nullptr,
                            .multiplexValue = 0,
                            .enumMap = nullptr,
                            .overrideCapable = true });

// ----------------------------------------------------

CanMessage eps300({ .name = "EPS300",
                    .idType = STANDARD,
                    .id = 0x300,
                    .dlc = 1,
                    .defaultFill = 0x00,
                    .comment = "From ECM, contains Cruise lights" });

CanSignal steeringLight({ .name = "SteeringLight",
                          .dataType = UNSIGNED,
                          .startBit = 7,  //byte0 bit 7
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Yellow steering wheel Light",
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

// --------------------------------------------------

CanMessage abs212({ .name = "ABS212",
                    .idType = STANDARD,
                    .id = 0x212,
                    .dlc = 7,
                    .defaultFill = 0x00,
                    .comment = "From ECM, contains Cruise lights" });

CanSignal dscTcEnable({ .name = "DSC_TC_Enable",
                          .dataType = UNSIGNED,
                          .startBit = 26,  //byte3 bit 3
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Enables the DSC and TC lights, neither will enable without this bit set", 
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

CanSignal absLight({ .name = "ABSLight",
                          .dataType = UNSIGNED,
                          .startBit = 35,  //byte4 bit 4
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Yellow ABS light", 
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

CanSignal brakeWarnLight({ .name = "BrakeWarningLight",
                          .dataType = UNSIGNED,
                          .startBit = 38,  //byte4 bit 7
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Red handbrake on or Brakes failed light", 
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

CanSignal DscOffLightn({ .name = "DSC_OffLight",
                          .dataType = UNSIGNED,
                          .startBit = 46,  //byte5 bit 4
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Yellow DSC OFF lights, signal inverted, i.e. 0 is light on, 1 if light off", 
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

CanSignal tcLight({ .name = "TractionControlLight",
                          .dataType = UNSIGNED,
                          .startBit = 44,  //byte5 bit 5
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Yellow traction control light, requires byte3 bit 3 to be set, overridden by flashing bit 6", 
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

CanSignal tcFlashingLight({ .name = "TractionControlFlashingLight",
                          .dataType = UNSIGNED,
                          .startBit = 45,  //byte5 bit 6
                          .bitLength = 1,
                          .endianness = BIG_ENDIAN,
                          .factor = 1,
                          .offset = 0,
                          .unit = "NA",
                          .comment = "Yellow traction control light flashing, requires byte3 bit 3 to be set, overrides steady light on bit 5", 
                          .min = 0,
                          .max = 1,
                          .defaultValue = 0,
                          .signalRole = NORMAL_SIGNAL,
                          .multiplexor = nullptr,
                          .multiplexValue = 0,
                          .enumMap = nullptr,
                          .overrideCapable = true });

// --------------------------------------------------

bool setDeviceSignals() {
  bool ok = true;
  // CanMessage engine201 CanSignal rpm, CanSignal speedo
  // CanMessage engine240 CanSignal rawECT
  // CanMessage engine420 CanSignal gaugeECT, CanSignal oilGauge "Low" "Ok" "Fault", CanSignal celOn,  CanSignal celFlashing, CanSignal coolantLight, CanSignal batteryLight, CanSignal oilLight
  // CanMessage engine650 CanSignal cruiseLight, CanSignal cruiseMainLight
  ok &= engine201.addSignal(rpm);
  ok &= engine201.addSignal(speedo);

  ok &= engine240.addSignal(rawECT);

  ok &= engine420.addSignal(gaugeECT);
  ok &= engine420.addSignal(oilGauge); //"Low" "Ok" "Fault"
  ok &= engine420.addSignal(celOn);
  ok &= engine420.addSignal(celFlashing);
  ok &= engine420.addSignal(coolantLight);
  ok &= engine420.addSignal(batteryLight);
  ok &= engine420.addSignal(oilLight);

  ok &= engine650.addSignal(cruiseLight);
  ok &= engine650.addSignal(cruiseMainLight);

  // CanMessage eps300 CanSignal steeringLight
  ok &= eps300.addSignal(steeringLight);

  // CanMessage abs212 CanSignal dscTcEnable, CanSignal absLight, CanSignal brakeWarnLight, CanSignal DscOffLightn, CanSignal tcLight, CanSignal tcFlashingLight
  ok &= abs212.addSignal(dscTcEnable);
  ok &= abs212.addSignal(absLight);
  ok &= abs212.addSignal(brakeWarnLight);
  ok &= abs212.addSignal(DscOffLightn);
  ok &= abs212.addSignal(tcLight);
  ok &= abs212.addSignal(tcFlashingLight);
  return ok;
}