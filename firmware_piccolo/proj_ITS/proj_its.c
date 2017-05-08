/*
 *     ___      _____ ___ ___
 *    /_\ \    / / __| __/ __|
 *   / _ \ \/\/ / (_ | _|\__ \
 *  /_/ \_\_/\_/ \___|___|___/
 *
 *  Projeto: Pistola Cirurgica  ITS-MC
 *  Desenvolvedor: Ânderson Ignácio da Silva
 *  Data: 27/07/16
 *  Plataforma: C2000 Launchpad (LAUNCHXL-F28027) - Piccolo TMS320F28027 MCU
 *  Descrição: Arquivo principal contendo todas as rotinas para funcionamento do equipamento
 *             O arquivo principal dos parâmetros do motor se chama user.h
 *             Porém deve-se atentar ao fato desse arquivo ter que ser copiado
 *             manualmente para o local de onde o projeto chama-o.
 *
 *  Lista de entradas e saídas:
 *   Gatilhos dos sensores hall:
 *     H1 = Pino ADCINB2 - 09 do J1
 *     H2 = Pino ADCINB4 - 10 do J1
 *   Drivers de saída para o motor:
 *     BOOSTXL-DRV8301
 *   Configuração de Jumpers para DEBUG:
 *     S4 - ON
 *     S1 - ON-ON-ON/UP-UP-UP
 *   Configuração de Jumpers para BOOT DA FLASH:
 *     S4 - ON
 *     S1 - ON-ON-OFF/UP-UP-DOWN
 *  Módulos utilizados:
 *     ADC - B2 e B4
 *     Timer 0
 *  Pendências possíveis para se fazer:
 *     Obtenção dos parâmetros de tensão de offset e corrent de offset através dos testes do lab2c
 *     Acionar o cálculo do Rs online ou da ténica de field weakining
 */

// the includes

// system includes
#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5


// **************************************************************************
// the globals
_iq angle_pu;                 //Variável que estima o ângulo do rotor (não utilizada até o momento)
MATH_vec2 phasor;             //Auxiliar para obtenção do ângulo do rotor

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;


#ifdef F2802xF
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif
HAL_Handle halHandle;

#ifdef F2802xF
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif
USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
#ifdef F2802xF
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif
CTRL_Obj ctrl;        //v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;


#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef F2802xF
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif

#endif

SVGENCURRENT_Obj svgencurrent;
SVGENCURRENT_Handle svgencurrentHandle;

// set the offset, default value of 1 microsecond
int16_t gCmpOffset = (int16_t)(1.0 * USER_SYSTEM_FREQ_MHz);

MATH_vec3 gIavg = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
uint16_t gIavg_shift = 1;


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;


/***************************************************************************
 *  Dados definidos por: Ânderson I. da Silva
***************************************************************************/
#define MIN_VALUE_HALL   _IQ(0.1)        //(Padrão 10%)  Valor mínimo percentual para sensibilização de pressionamento dos sensores de efeito hall
#define MAX_VALUE_HALL   _IQ(1.0)        //(Padrão 1 - IQ_24) Valor máximo do fundo de escala do sensor hall
#define SIZE_STEPS_kRPM  _IQ(1.0)        //(Padrão 1kRPM) Valor em kilo RPM de resolução de velocidade
#define SENSOR_CW        hallSensor1     //(Padrão H1) Define o sensor para o sentido de rotação horário
#define SENSOR_CCW       hallSensor2     //(Padrão H2) Define o sensor para o sentido de rotação anti-horário
#define MAX_kRPM_VALUE   _IQ(28.2)       //(Padrão 28.200RPM) Valor em kilo RPM de velocidade máxima da Pistola
#define ACCEL_IND_VALUE  _IQ(126.0)      //(Padrão 126 kRPM/s) Valor em kilo RPM por segundo da aceleração em acionamento individual dos sensores
#define CYCLES_ADC_READ  1               //(Padrão 1) Número de ciclos antes de reler o conversor ADC
#define ENABLE_ADC_HALL                  //(Padrão não comentado) Habilita ou desabilita os sensores hall da placa final
//#define ENABLE_MANUAL_SPEED              //(Padrão comentado) Habilita ou desabilita o controle manual do setpoint de velocidade
//#define ENABLE_LAUNCHPAD_POT_TEST      //(Padrão comentado) Habilita ou desabilita o controle de velocidade pelos adaptador no launchpad
#define N_ADC_FILTER     _IQ(2.0)        //Coeficiente do filtro por sw de pólo singular
#define MAX_ACCEL_VALUE  _IQ(126)        //(Padrão 126 kRPM/s)Maior valor real de aceleração possível de se aplicar ao motor, valor em kilo RPM por segundo da aceleração em acionamento duplo dos sensores
#define SIZE_STEPS_ACCEL _IQ(30)         //(Padrão 30kRPM/s) Valor em kilo RPM por segundo de resolução de aceleração do pressionamento duplo dos gatilhos
#define OFFSET_HALL_1    _IQ(0.55)       //(Padrão 0.55) Valor de offset para o valor ADC dos gatilhos do sensor Hall
#define OFFSET_HALL_2    _IQ(0.55)       //(Padrão 0.55) Valor de offset para o valor ADC dos gatilhos do sensor Hall
#define LIMIT_TORQUE     _IQ(9.15)       //(Padrão 9.15) Valor de corrente para o setpoint de torque do motor maxon
//#define DISABLE_TORQUE                 // (Padrão comentada) Desabilita controle de torque se não estiver comentada
//#define AXIS_FREE_ZERO_S               //(Padrão comentada - PERIGO - Descomentar PEGA FOGO) Macro que define o estado do eixo em velocidade zero, Comentada-eixo travado, Não comentada-eixo solto AVISO: Atualmente ao acionar esta opção a fonte pode entrar em modo proteção pelo solavanco do motor ao ligar o PWM
#define MASK_ADC_ENABLE                  //(Padrão comentada) Elimina a precisão do sensor AD através de mascaramento

_iq errorSpeed     = _IQ(0.0),
    ActualSpeedRef = _IQ(0.0),
    cursoHall      = _IQ(0.0),
    stepHallADC    = _IQ(0.0),
    resSpeed       = _IQ(0.0),
    incSpeed       = _IQ(0.0),
    speedRefOld    = _IQ(0.0),
    stepAccel      = _IQ(0.0),
    resAccel       = _IQ(0.0),
    incAccel       = _IQ(0.0),
    auxCalcTimer   = _IQ(0.0);

uint16_t delayReadADC        = 0;
uint32_t globalMachineCycles = 0;
uint32_t  setTimerToggle     = 0;

//Estrutura de variáveis para leitura dos valores dos sensores hall
typedef struct hallADCSensors{
  _iq hallSensor1;
  _iq hallSensor2;
  _iq hallSensor1Old;
  _iq hallSensor2Old;
  _iq meanDoubleTrig;
}hallADCSensors;

//Máquina de estados utilizada para controle de giro
typedef enum systemState{
  iddle,
  TriggerCW,
  TriggerCCW,
  TriggerDouble
}systemState;

_iq valor_hall_1;
_iq valor_hall_2;

systemState    mainState = iddle;

hallADCSensors STriggers = { _IQ(0.00),
                             _IQ(0.00) };

/***************************************************************************
 *  Fim de dados definidos por: Ânderson I. da Silva
***************************************************************************/

/***************************************************************************
 *  Funções definidas por: Ânderson I. da Silva
***************************************************************************/
//As funções são declaradas como "static inline" para que o compilador
//otimize o processo de síntese, alocando o código dentro da função principal

static inline void initCalcs(HAL_Handle handle){
  cursoHall                  = MAX_VALUE_HALL - MIN_VALUE_HALL;
  resSpeed                   = _IQdiv(MAX_kRPM_VALUE,SIZE_STEPS_kRPM);
  stepHallADC                = _IQdiv(cursoHall,resSpeed);
  gMotorVars.MaxAccel_krpmps = ACCEL_IND_VALUE;
  resAccel                   = _IQdiv(MAX_ACCEL_VALUE,SIZE_STEPS_ACCEL);
  stepAccel                  = _IQdiv(cursoHall,resAccel);

  HAL_enableTimer0Int(handle); //Habilita o TIMER0 utilizado no controle de rotação do pressionamento duplos dos gatilhos
}

static inline void HAL_readHallData(HAL_Handle handle)
{
  //Leitura dos conversores ADC descontando offset
  HAL_Obj *obj = (HAL_Obj *)handle;
  valor_hall_1 = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_8));
  valor_hall_2 = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_9));

  #ifndef ENABLE_MANUAL_SPEED
      #ifdef ENABLE_ADC_HALL
        STriggers.hallSensor1 = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_8))-OFFSET_HALL_1;
        STriggers.hallSensor2 = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_9))-OFFSET_HALL_2;

        STriggers.hallSensor1 = _IQmpy(STriggers.hallSensor1,_IQ(16.67));
        STriggers.hallSensor2 = _IQmpy(STriggers.hallSensor2,_IQ(16.67));

        #ifdef MASK_ADC_ENABLE //0.XXX000000000000
          //Máscara para o conversor ADC para eliminar ruídos
          STriggers.hallSensor1 = _IQmpy(STriggers.hallSensor1,_IQ(100));
          STriggers.hallSensor2 = _IQmpy(STriggers.hallSensor2,_IQ(100));

          STriggers.hallSensor1 = STriggers.hallSensor1-_IQfrac(STriggers.hallSensor1);
          STriggers.hallSensor2 = STriggers.hallSensor2-_IQfrac(STriggers.hallSensor2);

          STriggers.hallSensor1 = _IQmpy(STriggers.hallSensor1,_IQ(0.01));
          STriggers.hallSensor2 = _IQmpy(STriggers.hallSensor2,_IQ(0.01));
        #endif
      #endif

      #ifdef ENABLE_LAUNCHPAD_POT_TEST
          STriggers.hallSensor1 = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_8));
          STriggers.hallSensor2 = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_9));
      #endif
  #endif

}

static inline void setState()
{
  //Computa o erro absoluto do setpoint de velocidade
  errorSpeed = gMotorVars.SpeedRef_krpm - gMotorVars.Speed_krpm;
  errorSpeed = _IQabs(errorSpeed);

  switch(mainState){
    case iddle:
      if (STriggers.SENSOR_CW >= MIN_VALUE_HALL && STriggers.SENSOR_CCW <= MIN_VALUE_HALL)
      {
        mainState = TriggerCW;
      }
      else if (STriggers.SENSOR_CCW >= MIN_VALUE_HALL && STriggers.SENSOR_CW <= MIN_VALUE_HALL)
      {
        mainState = TriggerCCW;
      }
      else if(STriggers.SENSOR_CCW >= MIN_VALUE_HALL && STriggers.SENSOR_CW >= MIN_VALUE_HALL)
      {
        mainState = TriggerDouble;
      }
      else
      {
        #ifndef DISABLE_TORQUE
          gMotorVars.IdRef_A = _IQ(0);
          gMotorVars.IqRef_A = _IQ(0);
        #endif

        #ifndef ENABLE_MANUAL_SPEED
          gMotorVars.SpeedRef_krpm = _IQ(0.0);
        #endif ENABLE_MANUAL_SPEED

        mainState = iddle;
        #ifdef AXIS_FREE_ZERO_S
          HAL_disablePwm(halHandle);
        #endif
      }
    break;
    case TriggerCW:
      #ifdef AXIS_FREE_ZERO_S
        HAL_enablePwm(halHandle);
      #endif
      globalMachineCycles = 0;

      if(STriggers.SENSOR_CW > MAX_VALUE_HALL){
          STriggers.SENSOR_CW = MAX_VALUE_HALL;
      }

      gMotorVars.MaxAccel_krpmps = ACCEL_IND_VALUE;
      incSpeed                   = _IQdiv(STriggers.SENSOR_CW,stepHallADC);
      resSpeed                   = incSpeed-_IQfrac(incSpeed);
      incSpeed                   = _IQmpy(resSpeed,SIZE_STEPS_kRPM);

      #ifndef DISABLE_TORQUE
        gMotorVars.IdRef_A = LIMIT_TORQUE;
        gMotorVars.IqRef_A = LIMIT_TORQUE;
      #endif

      if(incSpeed >= MAX_kRPM_VALUE)
          incSpeed = MAX_kRPM_VALUE;

      if (incSpeed != gMotorVars.SpeedRef_krpm){
        gMotorVars.SpeedRef_krpm = incSpeed;
        speedRefOld = incSpeed;
      }

      mainState = iddle;
    break;
    case TriggerCCW:
      #ifdef AXIS_FREE_ZERO_S
        HAL_enablePwm(halHandle);
      #endif
      globalMachineCycles = 0;

      if(STriggers.SENSOR_CCW > MAX_VALUE_HALL){
          STriggers.SENSOR_CCW = MAX_VALUE_HALL;
      }

      gMotorVars.MaxAccel_krpmps = ACCEL_IND_VALUE;
      incSpeed                   = _IQdiv(STriggers.SENSOR_CCW,stepHallADC);
      resSpeed                   = incSpeed-_IQfrac(incSpeed);
      incSpeed                   = -(_IQmpy(resSpeed,SIZE_STEPS_kRPM));

      #ifndef DISABLE_TORQUE
        gMotorVars.IdRef_A = LIMIT_TORQUE;
        gMotorVars.IqRef_A = LIMIT_TORQUE;
      #endif

      if(incSpeed >= MAX_kRPM_VALUE)
          incSpeed = MAX_kRPM_VALUE;

      if (incSpeed != gMotorVars.SpeedRef_krpm){
        gMotorVars.SpeedRef_krpm = incSpeed;
        speedRefOld = incSpeed;
      }
      mainState = iddle;
    break;
    case TriggerDouble:
 //      O princípio de funcionamento da comutação de velocidade é simples, utilizando o conceito de MRUV
 //      A velocidade em um MRUV é dada por V=V0+aT, logo temos que o tempo necessário para que um objeto
 //      alcance determinada velocidade é de T=(V-V0)/a, assim utilizando desta lógica sempre que dois ga
 //      tilhos são pressionados, calcula-se esse novo tempo o qual determina o setpoint do TIMER0.
 //      Considerando que o TIMER0 é disparado a cada 1ms, utiliza-se uma variável inteira para contar,
 //      aproximadamente o número de estouros que certa velocidade deve-se manter antes de inverter o sen
 //      tido de rotação. Para o cálculo do tempo, considera-se sempre o tempo necessário  para  que o mo
 //      tor atinga a velocidade máxima de giro configurada, assim como no exemplo abaixo:
 //
 //      Exemplo 1:
 //        Gatilhos:
 //        (1)------|     = 45%
 //        (2)---------|  = 65%
 //
 //        Valor dos gatilhos = 55%
 //        MAX_kRPM_VALUE     = 15.6kRPM
 //        MAX_ACCEL_VALUE    = 126 kRPM/s
 //
 //                ^ V(kRPM)
 //                |
 // +MAX_kRPM_VALUE|     /\          /\
 //                |    /  \        /  \
 //                |___/____\______/____\__________ T
 //                |  /      \    /      \
 //                | /        \  /        \
 // -MAX_kRPM_VALUE|/          \/          \
 //                |
 //                      |------|
 //                        2x setTimerToggle = 2x MAX_kRPM_VALUE/(.65*MAX_ACCEL_VALUE) = 380ms
 //                      Logo a cada 380ms o setpoint de velocidade será chaveado de um extremo a outro;

 //      Exemplo 2:
 //        Gatilhos:
 //        (1)--|           = 25%
 //        (2)-----------|  = 75%

 //        Valor dos gatilhos = 50%
 //        MAX_kRPM_VALUE     = 15.6kRPM
 //        MAX_ACCEL_VALUE    = 126 kRPM/s

 //                ^ V(kRPM)
 //                |
 // +MAX_kRPM_VALUE|     /\          /\
 //                |    /  \        /  \
 //                |___/____\______/____\__________ T
 //                |  /      \    /      \
 //                | /        \  /        \
 // -MAX_kRPM_VALUE|/          \/          \
 //                |
 //                      |------|
 //                        2x setTimerToggle = 2x MAX_kRPM_VALUE/(.50*MAX_ACCEL_VALUE) = 495ms
 //                      Logo a cada 495ms o setpoint de velocidade será chaveado de um extremo a outro;

      STriggers.meanDoubleTrig   = _IQdiv((STriggers.SENSOR_CW+STriggers.SENSOR_CCW),_IQ(2.0));
      incAccel                   = _IQdiv(STriggers.meanDoubleTrig,stepAccel);
      resAccel                   = incAccel-_IQfrac(incAccel);
      incAccel                   = _IQmpy(resAccel,SIZE_STEPS_ACCEL);

      if(incAccel >= MAX_ACCEL_VALUE){
         incAccel = MAX_ACCEL_VALUE;
      }

      gMotorVars.MaxAccel_krpmps = incAccel;

      auxCalcTimer               = _IQdiv(MAX_kRPM_VALUE,incAccel);
       setTimerToggle             = _IQtoF(auxCalcTimer)*50; //Multiplica-se por 2k porque é 2x 1000ms de conversão conforme briefing anterior
      mainState = iddle;
    break;
    default:
    break;
  }
//      setTimerToggle             = _IQtoF(auxCalcTimer)*550; //Alterado para teste
}

interrupt void timer0ISR(void) {
  HAL_acqTimer0Int(halHandle);

  if(mainState == TriggerDouble)
    if (globalMachineCycles >= setTimerToggle)
    {
      globalMachineCycles = 0;
      toggleSetSpeed();
    }
    else
      globalMachineCycles += 1;
  return;
}

void toggleSetSpeed(){
  if (gMotorVars.SpeedRef_krpm > 0)
    gMotorVars.SpeedRef_krpm = -(MAX_kRPM_VALUE);
  else
    gMotorVars.SpeedRef_krpm = MAX_kRPM_VALUE;
}
/***************************************************************************
 *  Fim de funções definidas por: Ânderson I. da Silva
***************************************************************************/



void main(void)
{
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);

  #ifdef F2802xF
    //copy .econst to unsecure RAM
    if(*econst_end - *econst_start)
      {
        memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
      }

    //copy .switch ot unsecure RAM
    if(*switch_end - *switch_start)
      {
        memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
      }
  #endif

  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);      //v1p6 format (06xF and 06xM devices)
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl)); //v1p7 format default
#endif

  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);


  // Initialize and setup the 100% SVM generator
  svgencurrentHandle = SVGENCURRENT_init(&svgencurrent,sizeof(svgencurrent));

  // setup svgen current
  {
    float_t minWidth_microseconds = 2.0;
    uint16_t minWidth_counts = (uint16_t)(minWidth_microseconds * USER_SYSTEM_FREQ_MHz);

    SVGENCURRENT_setMinWidth(svgencurrentHandle, minWidth_counts);
    SVGENCURRENT_setIgnoreShunt(svgencurrentHandle, use_all);
  }


  // set overmodulation to maximum value
  gMotorVars.OverModulation = _IQ(MATH_TWO_OVER_THREE);


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
  // turn on the DRV8305 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8305 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();


  initCalcs(halHandle);
  for(;;)
  {
    // Waiting for enable system flag to be set
    //while(!(gMotorVars.Flag_enableSys));
    #ifndef DISABLE_TORQUE
    gMotorVars.IdRef_A = LIMIT_TORQUE;
    gMotorVars.IqRef_A = LIMIT_TORQUE;
    #endif
    gMotorVars.Flag_enableUserParams = true;
    gMotorVars.Flag_enableSys = true; //Consideramos que o sistema é inicializado automaticamente
    gMotorVars.Flag_Run_Identify = true; //Motor já identificado
//    gMotorVars.Flag_enableRsRecalc = true;
    // Enable the Library internal PI.  Iq is referenced by the speed PI now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);

    //Loop principal - Tecnicamente não deve sair deste laço
    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {

        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams); //Habilita o uso de parâmetros diretos do user.h

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc); //Habilita o cálculo online da resistência Rs do motor - Desativado por padrão

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

            if(flag_ctrlStateChanged)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                    }

                    // Return the bias value for currents
                    gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
                    gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
                    gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);

                    // Return the bias value for voltages
                    gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
                    gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
                    gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);

                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }

              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
            _iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle),CTRL_getId_ref_pu(ctrlHandle));


            //Set the maximum current controller output for the Iq and Id current controllers to enable
            //over-modulation.
            //An input into the SVM above 1/SQRT(3) = 0.5774 is in the over-modulation region.  An input of 0.5774 is where
            //the crest of the sinewave touches the 100% duty cycle.  At an input of 2/3, the SVM generator
            //produces a trapezoidal waveform touching every corner of the hexagon
            CTRL_setMaxVsMag_pu(ctrlHandle,gMotorVars.OverModulation);

            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            // set the Id reference
            CTRL_setId_ref_pu(ctrlHandle, _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;

              USER_calcPIgains(ctrlHandle);

              // initialize the watch window kp and ki current values with pre-calculated values
              gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
              gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // initialize the watch window kp and ki values with pre-calculated values
            gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle,CTRL_Type_PID_spd);
            gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle,CTRL_Type_PID_spd);


            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandle);
          }


        // update Kp and Ki gains
        updateKpKiGains(ctrlHandle);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif
        if (delayReadADC == CYCLES_ADC_READ)
        {
          // Leitura dos valores dos gatilhos da pistola
          HAL_readHallData(halHandle);
          // Função principal para operar a pistola
          setState();
          delayReadADC = 0;
        }
        else{
          delayReadADC = delayReadADC + 1;
        }

      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;

  } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{
  SVGENCURRENT_IgnoreShunt_e ignoreShuntThisCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);

  // toggle status LED
  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }


  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);


  // run the current reconstruction algorithm
  SVGENCURRENT_RunRegenCurrent(svgencurrentHandle, (MATH_vec3 *)(gAdcData.I.value));

  gIavg.value[0] += (gAdcData.I.value[0] - gIavg.value[0])>>gIavg_shift;
  gIavg.value[1] += (gAdcData.I.value[1] - gIavg.value[1])>>gIavg_shift;
  gIavg.value[2] += (gAdcData.I.value[2] - gIavg.value[2])>>gIavg_shift;

  if(ignoreShuntThisCycle == ignore_ab)
  {
    gAdcData.I.value[0] = gIavg.value[0];
    gAdcData.I.value[1] = gIavg.value[1];
  }
  else if(ignoreShuntThisCycle == ignore_ac)
  {
    gAdcData.I.value[0] = gIavg.value[0];
    gAdcData.I.value[2] = gIavg.value[2];
  }
  else if(ignoreShuntThisCycle == ignore_bc)
  {
    gAdcData.I.value[1] = gIavg.value[1];
    gAdcData.I.value[2] = gIavg.value[2];
  }


  // run the controller
  CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData);


  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  // run the current ignore algorithm
  {
    uint16_t cmp1 = HAL_readPwmCmpA(halHandle,PWM_Number_1);
    uint16_t cmp2 = HAL_readPwmCmpA(halHandle,PWM_Number_2);
    uint16_t cmp3 = HAL_readPwmCmpA(halHandle,PWM_Number_3);
    uint16_t cmpM1 = HAL_readPwmCmpAM(halHandle,PWM_Number_1);
    uint16_t cmpM2 = HAL_readPwmCmpAM(halHandle,PWM_Number_2);
    uint16_t cmpM3 = HAL_readPwmCmpAM(halHandle,PWM_Number_3);

    // run the current ignore algorithm
    SVGENCURRENT_RunIgnoreShunt(svgencurrentHandle,cmp1,cmp2,cmp3,cmpM1,cmpM2,cmpM3);
  }

  {
    int16_t minwidth = SVGENCURRENT_getMinWidth(svgencurrentHandle);
    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(halHandle,ignoreShuntNextCycle,minwidth,gCmpOffset);
  }

  // setup the controller
  CTRL_setup(ctrlHandle);

  CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

 // generate the motor electrical angle      //Adicionado por anderson
 angle_pu = EST_getAngle_pu(obj->estHandle); //Adicionado por anderson

  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // read Vd and Vq vectors per units
  gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
  gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

  // calculate vector Vs in per units
  gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

  // read Id and Iq vectors in amps
  gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
  gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  // calculate vector Is in amps
  gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
  }

  return;
} // end of updateKpKiGains() function


//@} //defgroup
// end of file
