//======================================================================================================
//                                                                                       							//
//   Descripción del programa:                                                                 		 		//
//                                                                                       							//
//                                                                                       							//
//                                                                                       							//
//                                                                                       							//
//      Autores:                                                                           						//
//                                                                                       							//
//                                                                                      							//
//      Micro:                  DSPIC30F4011                                                         	//
//      Frecuencia del oscilador:   120 MHz                                                           //
//                                                                                          					//
//======================================================================================================

#include <30f4011.h>
#fuses FRC_PLL16,NOPROTECT,NOWDT,NOPUT
#device adc=10
#use delay(clock=120000000)
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#use RS232(BAUD=115200,BITS=8,PARITY=N,XMIT=PIN_C14,RCV=PIN_C13,STREAM=PC))// UART
#use fast_io(B)
#use fast_io(C)
#use fast_io(D)
#use fast_io(E)
#use fast_io(F)

//====================================================================================================================
//                                    Declaración de variables                                 //
//====================================================================================================================

#define Button PIN_B6
#define LED_Izquierdo PIN_B7
#define LED_Derecho PIN_B8
#define Cambio_Linea PIN_F5
#define RX PIN_F4
#define Pot sAN4

//Pines Motores OC3 OC4
#define AD  pin_E5
#define BD pin_E4
#define AI pin_E2 
#define BI pin_E3
/*----------- Motores---------*/
int16 UL=000;  // Velocidad de motor Iquierdo [0 1]
int16 UR=000;  // Velocidad de motor Derecho [0 1]
int16 PWM_max=10000;   // Maximo valor posible de PWM, depende de configuración PWM
int16 PWM_min=-10000;  // Mínimo valor posible de PWM, depende de configuración PWM
/*----------------------------*/
float Smin=400.0;
float Smax=1023.0;
float SenVal[16];
float sensors_average=0.0;
float sensors_sum=0.0;
int x=0;
int on_line=0;
float line_1=0.0;
float line=0.0;

int8 const N[16][4] ={  {0,0,0,0},
                        {0,0,0,1},
                        {0,0,1,0},
                        {0,0,1,1},
                        {1,0,0,0},
                        {1,0,0,1},
                        {1,0,1,0},
                        {1,0,1,1},
                        {0,1,0,0},
                        {0,1,0,1},
                        {0,1,1,0},
                        {0,1,1,1},
                        {1,1,0,0},
                        {1,1,0,1},
                        {1,1,1,0},
                        {1,1,1,1}};


/* pines
PIN_RB0 ------ S0
PIN_RB1 ------ S1
PIN_RB2 ------ DP
PIN_RB3 ------ S3
PIN_RB4 ------ S2
*/

//====================================================================================================================
//                                    Funciones y procedimientos                                 //
//====================================================================================================================

float Sensores(int8 Num_Sensor=0)
{
output_bit(PIN_B3,N[Num_Sensor][0]);
output_bit(PIN_B2,N[Num_Sensor][1]);
output_bit(PIN_B1,N[Num_Sensor][2]);
output_bit(PIN_B0,N[Num_Sensor][3]);
delay_us(100);
int8 ValSensor=0;
set_adc_channel(5);        //Establecemos el canal de lectura analogica
float medicion=read_adc ();            // Hace conversi?n AD
//if (medicion>800){
//Valsensor=0;
//}else{
//Valsensor=1;}
//printf (" %f \r ",medicion);     
return medicion;
}

const int Numer_Samples=1; // Muestras por sensor, mas muestras para mejor sensado.
const int NumSensors=16;  // Numero de sensores

float map(float value, float minSen, float maxSen, float y_min, float y_max)//funcion de normalizacion
{
  float Vcal= (((Value-minSen)*(y_max-y_min))/(maxSen-minSen)) -y_min; // ecuacion de normalizacion
  if( Vcal>y_max){Vcal=y_max;}
  if( Vcal<y_min){Vcal=y_min;}
  return  Vcal;
}

float Minimos,Maximos;

float Max[16] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float Min[16] = {1023.0,1023.0,1023.0,1023.0,1023.0,1023.0,1023.0,1023.0,1023.0,1023.0,
				1023.0,1023.0,1023.0,1023.0,1023.0,1023.0};

// Funcion que calibra sensores, encuentra valores maximos y minimos.
void Calibrar_sensores() {
	int8 Columna,fila;                     // Almacenamos en una Matriz la lectura
	float SetP=400.0;
	float valor=0.0;
	float valor2=0.0;
	int16 i=0;
	int8 x=0;
	int8 y=0;
	float vsensor = 0.0;	

		while(1){
		valor=Sensores(0);
		valor2=Sensores(15);
		if(valor>Setp && valor2>Setp){
			for(y=0;y<25;y++){
			for(x=0;x<16;x++){
				//	delay_ms(10);
				for(i=0;i<100;i++){
				
					vsensor = Sensores(x);
					if(vsensor>Max[x]){
						Max[x] = vsensor;
					}
	
					if(vsensor<Min[x] && vsensor!=1023.0){
						Min[x] = vsensor;
					}	
				}				
			}	
			}
			for(x=0;x<16;x++){
				fprintf (PC,"||%f\r",Max[x]);
			}
	 		fprintf (PC,"||%f\r\n",0.0);
	
			for(x=0;x<16;x++){
				fprintf (PC,"||%f\r",Min[x]);
			}
	
			fprintf (PC,"||%f\r\n",0.0);
			break;
		}	
	} 

output_HIGH(Led_Derecho);
output_HIGH(Led_Izquierdo);
delay_ms(100);
output_LOW(Led_Derecho);
output_LOW(Led_Izquierdo); 
delay_ms(100); 
output_HIGH(Led_Derecho);
output_HIGH(Led_Izquierdo);
}	

float Calcular_Linea(){ // Funcion que calcula la posicion de la linea
	on_line=0;
	float Vr=0;
	for(x=0;x<16;x++){
		SenVal[x]= (Sensores(x));  // lee el pin
   
		/*linea negra 1.0 linea blanca 0.0*/
		Vr=1.0 - map(SenVal[x],Min[x],Max[x],0.0,1.0); // Normaliza el valor leido //Vs,Vm,Vmx,0,1000 []Mx,MIn
  		//if(x==8){Vr=0.0;} 
  		// printf (" %f \r ",Vr);     
   		//Vr=SenVal[x];
		sensors_average+=(Vr * (x));  // Numerador de la ecuacion
		sensors_sum +=Vr;                   // Denominador de la ecuacion
		if (Vr>1){on_line=1;}           // Identificador, pero ni idea de xq.
	}
  
  //printf (" %f \r ",sensors_average);
  //printf (" %f \r ",sensors_sum);
  //printf (" %i \r ",sensors_average);
  //printf (" %i \r  ",sensors_sum);
	if(sensors_average==0 || sensors_sum==0){
		sensors_average=1,sensors_sum=1;
  	}
  
	//fprintf (PC,"%i\n",Vr);
	line=(sensors_average/sensors_sum);      // Calculo del valor de linea.
	// agregado por Harold
	// agregado por Harold
	if (sensors_sum>1){
	on_line=1;
	line_1=line;  
	}

	else{
		on_line=0;
	}

	if (on_line==0){
		line = line_1;
	}
  
	sensors_average=0;   // Elimina valores
	sensors_sum=0;
	Vr=0;
	return line;     // Retorna Calculo de Linea.
}

int8 Color_linea=0;
int8 IR=0;
float32 V_pot=0;
int8 pulsador=0;  
int8 i=0;

 // Accion sobre motores
 
void Motors(int16 ul,int16 ur){
	if (ur>PWM_max){UR=PWM_max;}
	if (ul>PWM_max){UL=PWM_max;}
	if (ur<PWM_min){UR=PWM_min;}
	if (ul<PWM_min){UL=PWM_min;}

	if(ur>=0){ //OC4
		output_high(AD);
		output_low(BD);
		set_pwm_duty(4,abs(ur));  // 2ms on status
	}
	else{  
		output_low(AD);
		output_high(BD);
		set_pwm_duty(4,abs(ur));  // 2ms on status
	}
	if(UL>=0){
		output_high(AI);
		output_low(BI);
		set_pwm_duty(3,abs(ul));  // 2ms on status
	}
	else{
		output_low(AI);
		output_high(BI);
		set_pwm_duty(3,abs(ul));  // 2ms on status
	}
}
  
  
void init_Motors(){
	setup_timer3(TMR_INTERNAL | TMR_DIV_BY_8, 65500);
	setup_compare(1,COMPARE_PWM | COMPARE_TIMER3 );
	set_pwm_duty(1,4000); //Minimo 40000 para encender
	delay_ms(1500);
	
	setup_timer2(TMR_INTERNAL | TMR_DIV_BY_1, PWM_max );
	setup_compare(3,COMPARE_PWM | COMPARE_TIMER2 );
	setup_compare(4,COMPARE_PWM | COMPARE_TIMER2 );
	set_pwm_duty(3,0000);
	set_pwm_duty(4,0000);
	//ENABLE_INTERRUPTS(INTR_GLOBAL);
}

int16 V_Max=0, U_Max, U_Min;
float U,Up,Ud,Ud_1,error,error_1,linea;
int16 Kp,Kd;
int16 U_Base; // Se define en función: Tunning
int16 Tur;

void Tunning(){
	Tur=5000;
	Kp=3.5*1000.0;
	float Td=4.6;
	Kd=Kp*Td;
	V_Max=4000;//0.25     // Determina la velocidad del Robot
	U_Max=PWM_max;
	U_Min=PWM_min;
}

void OptimalSpeed(){
	// Sección Guarant Control del Paper		
	int16 delta;
	if(UR>U_MAX){
		delta  = U_MAX - UR;
	    U_Base = U_Base + delta;
	    UL=U_Base - 0.5*U; // Accion sobre motor Izquierdo
	    UR=U_Base + 0.5*U; // Accion sobre motor Derecho
	}

	if(UL>U_MAX){
		delta  = U_MAX - UL;
		U_Base = U_Base + delta;
		UL=U_Base - 0.5*U; // Accion sobre motor Izquierdo
		UR=U_Base + 0.5*U; // Accion sobre motor Derecho
	}
}

void PID(){
	// Accion proporcional
	Up= Kp*error;
	// Accion Derivativa
	Ud = Kd*(error-error_1);
	// Accion de Control
	U=Up+Ud;
	// fprintf (PC,"||%f\r\n",U);
	error_1 = error;
	Ud_1=Ud; 
}

//====================================================================================================================
//                                                                                   						//
//                                       PROGRAMA PRINCIPAL                                    				//
//                                                                                   						//
//====================================================================================================================

void main(){
	set_tris_B(0b000100000);
	set_tris_F(0xFF);
	set_tris_E(0x00);
	set_tris_D(0x00);
	fprintf (PC,"Hello World, My Name is Mustang!!!!! %i\r\n",1);
	setup_adc(ADC_CLOCK_INTERNAL);   // Configura el conversor
	setup_adc_ports( sAN5 );  // Configura el conversor
	init_Motors();
	Tunning();
	output_HIGH(Led_Derecho);
	output_HIGH(Led_Izquierdo);

	// Boton o Rx de arranque
	while(1){
	    if (input(Button)==1){
			output_LOW(Led_Derecho);
			output_LOW(Led_Izquierdo);
		
			Calibrar_sensores();
	    	break;
	    }
	}

	while(1){
		linea=Calcular_Linea();
		printf (" %f\r\n",linea);
		error=8.0 - linea;
	
		if (error>0){
			output_HIGH(LED_Izquierdo);
			output_LOW(LED_Derecho);
		}
	
		else{
			output_LOW(LED_Izquierdo);
			output_HIGH(LED_Derecho);
		}
	
		PID(); // Determina accion de control U
		U_Base =3000; // Valor asignado en la funcion: Tunning
	   
		UL=U_Base - 0.5*U; // Accion sobre motor Iquierdo
		UR=U_Base + 0.5*U; // Accion sobre motor Derecho
		OptimalSpeed();
	
	   	//UR=UL=00;
		Motors(UL,UR);

	 // printf (" %f %f %f \r\n",linea,Up,Ud);
	 // printf (" %f\r\n",linea);  
	/*if (input(RX)==0)
	    {
	output_LOW(Led_Derecho);
	output_LOW(Led_Izquierdo);
	Motors(0000,0000);
	      delay_ms(2000);
	      break;
	    }
	*/
	
	// fprintf (PC,"%i\r\n",Calcular_Linea());
	}
}
   
   



