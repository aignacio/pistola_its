# Pistola cirurgica - Controle de motor brushless</br>

__Desenvolvedor:__ Ânderson Ignacio da Silva</br>
Este projeto é composto de:
* Firmware específico desenvolvido para controle de um motor brushless sensorless através de estimadores

__Dispositivos alvos:__
* Motor maxon [386676](http://www.maxonmotor.com/medias/sys_master/root/8821064269854/16-247-EN.pdf)
* Microcontrolador Piccolo TMS320F28027F + DRV8301 - Instaspin FOC

**Requisitos necessários:**
* CCS Studio 6.X ou superior
* Motorware 1.16 - Deve ser esta versão

__Descrição de utilização:__
Para utilização do código fonte, deve-se atentar ao fato de que tanto o motorware **1.16** quanto o CCS studio devem estar instalados na máquina de desenvolvimento. Uma vez instalados, deve-se importar o projeto que está localizado em **firmware_piccolo/proj_ITS** através do CCS, e copiar os arquivos **hal.h** e **user.h** manualmente para as pastas **C:\ti\motorware\motorware_1_01_00_16\sw\modules\hal\boards\boostxldrv8301_revB\f28x\f2802x\src** e **C:\ti\motorware\motorware_1_01_00_16\sw\solutions\instaspin_foc\boards\boostxldrv8301_revB\f28x\f2802xF\src** manualmente. Também é necessário verificar se na importação do projeto pelo CCS os arquivos **user.c** e **hal.c** estão idênticos aos que estão presentes na pasta **firmware_piccolo/proj_ITS**. Logo após deve-se compilar todo o projeto e enviar o software a placa. Por padrão este firmware possui três modos de operação, **direto, reverso e alternado** onde em cada motor o sentido de giro e rotação são determinados pelos gatilhos dos sensores hall_1 e hall_2. Também estão presentes três macros nas quais selecionam três modos distintos de teste:</br>
__1)define ENABLE_ADC_HALL__</br>
__2)define ENABLE_MANUAL_SPEED__  
__3)define ENABLE_LAUNCHPAD_POT_TEST__</br>
O modo 1) habilita os sensores hall originais presentes na placa final de desenvolvimento, enquanto o modo 2) habilita o controle de velocidade apenas pelo modo de depuração da placa e no modo 3) temos o controle dos sensores através de uma placa adaptadora com potênciometros.
