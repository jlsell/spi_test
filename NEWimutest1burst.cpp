#include <bcm2835.h>
#include <iostream>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <bitset>
#include <ctime>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sys/time.h>
#include <cstdlib>

using std::cout;
using std::hex;
using std::dec;
using std::cin;
using std::cerr;
using std::endl;
using std::bitset;
using std::ofstream;
using std::tm;  //not sure if needed
using std::difftime;
using std::time;
using std::time_t;  //not sure if needed

const timespec stall={0,9};

const timespec burst_stall={0,1024};

namespace Register { //Inputting lower byte register addresses.  to get to the upper byte, add 1 to the value given
    enum register_t {
        FLASH_CNT	 = 0x0000,	//Flash memory write count
        SUPPLY_OUT	 = 0x0200,	//Power supply measurement
        GYRO_OUT	 = 0x0400,	//X-axis gyroscope output
        //YGYRO_OUT	 = 0x0600,	//NA
        //ZGYRO_OUT	 = 0x0800,	//NA
        XACCL_OUT	 = 0x0A00,	//X-axis accelerometer output
        YACCL_OUT	 = 0x0C00,	//Y-axis accelerometer output
        ZACCL_OUT	 = 0x0E00,	//Z-axis accelerometer output
        TEMP_OUT	 = 0x1000,	//Temperature output
        PITCH_OUT	 = 0x1200,	//Pitch measurement
        ROLL_OUT	 = 0x1400,	//Roll measurement
        AUX_ADC		 = 0x1600,	//Temperature output
        // AUX_ADC	 = 0x1800,	//Auxiliary ADC measurement
        GYRO_OFF	 = 0x1A00,	//X-axis gyroscope bias offset factor
        //YGYRO_OFF	 = 0x1C00,	//Y-axis gyroscope bias offset factor
        //ZGYRO_OFF	 = 0x1E00,	//Z-axis gyroscope bias offset factor
        XACCL_OFF	 = 0x2000,	//X-axis acceleration bias offset factor
        YACCL_OFF	 = 0x2200,	//Y-axis acceleration bias offset factor
        ZACCL_OFF	 = 0x2400,	//Z-axis acceleration bias offset factor
        XMAGN_HIF	 = 0x2600,	//X-axis magnetometer, hard-iron factor
        YMAGN_HIF	 = 0x2800,	//Y-axis magnetometer, hard-iron factor
        ZMAGN_HIF	 = 0x2A00,	//Z-axis magnetometer, hard-iron factor
        XMAGN_SIF	 = 0x2C00,	//X-axis magnetometer, soft-iron factor
        YMAGN_SIF	 = 0x2E00,	//Y-axis magnetometer, soft-iron factor
        ZMAGN_SIF	 = 0x3000,	//Z-axis magnetometer, soft-iron factor
        GPIO_CTRL	 = 0x3200,	//Auxiliary digital input/output control
        MSC_CTRL	 = 0x3400,	//Miscellaneous control
        SMPL_PRD	 = 0x3600,	//Internal sample period (rate) control
        SENS_AVG	 = 0x3800,	//Dynamic range and digital filter control
        SLP_CNT	 = 0x3A00,	//Sleep mode control
        DIAG_STAT	 = 0x3C00,	//System status
        GLOB_CMD	 = 0x3E00,	//System command
        ALM_MAG1	 = 0x4000,	//Alarm 1 amplitude threshold
        ALM_MAG2	 = 0x4200,	//Alarm 2 amplitude threshold
        ALM_SMPL1	 = 0x4400,	//Alarm 1 sample size
        ALM_SMPL2	 = 0x4600,	//Alarm 2 sample size
        ALM_CTRL	 = 0x4800,	//Alarm control
        AUX_DAC	 = 0x4A00,	//Auxiliary DAC data
        UPPER_BYTE	 = 0x0100,	//Add to address to access upper byte
        LOWER_BYTE	 = 0x0000,	//Add to address to access LOWER byte
        MSB_1	 = 0x8000,	//Use to change MSB to 1
        PROD_ID	 = 0x5600	//The register containing the product identification
    };
    
    uint16_t read_reg(int reg){ //this function simply casts the int type to a uint16_t type
        uint16_t out=(uint16_t)reg;
        return out;
    };
    uint16_t write_reg(int reg, int byte){  //This function casts the int type to a uint16_t type, sets the msb to 1, and sets the proper byte
        uint16_t out=(uint16_t)reg+(uint16_t)byte;
        out=out | MSB_1;
        return out;
    };
    void uint_to_char(uint16_t cmd, char* buf){  //this function composes the buffer command that is then to be sent to the device
        char temp[]={(char)((uint16_t)(cmd>>8)), (char)((uint16_t)(cmd<<8)>>8)};
        buf=temp;
    };
    uint16_t char_to_uint(char* buf){  //this function composes the output from the buffer
        uint16_t out = ((uint16_t)buf[0]<<8)+(uint16_t)buf[1];
        return out;
    };
}

using namespace Register;

namespace filter_set {//creating functions for setting the filter
    enum range {//inputting values for the gyro range
        pm300	 = 0x0004,
        pm150	 = 0x0002,
        pm75	 = 0x0001
    };
    
    void rangeset( int range_to_set,char* buf){//This function sets the gyro range on the IMU
        uint16_t command= write_reg(SENS_AVG,UPPER_BYTE)|range_to_set;
        uint_to_char(command,buf);  //converting command to char*
        bcm2835_spi_transfern(buf, sizeof(buf));  //writing command
        nanosleep(&stall,NULL); //stall period
    };
    void tapset(int ntaps,char* buf){//This function sets the number of filter taps on the imu.  ntaps must be able to be expressed as ntaps=2^N
        uint16_t N=log(ntaps)/log(2);
        uint16_t command= write_reg(SENS_AVG,LOWER_BYTE)|N;
        uint_to_char(command,buf);  //converting command to char*
        bcm2835_spi_transfern(buf, sizeof(buf));  //writing command
        nanosleep(&stall,NULL); //stall period
    };
    void readfilter(uint16_t* output,char* buf){//This function reads the current values in the SENS_AVG register.  The output is unedited
        uint16_t command = read_reg(SENS_AVG);
        uint_to_char(command,buf);  //converting command to char*
        bcm2835_spi_transfern(buf, sizeof(buf));  //writing command
        nanosleep(&stall,NULL); //stall period
        bcm2835_spi_transfern(buf, sizeof(buf));  //reading output
        nanosleep(&stall,NULL); //stall period for next command
        *output=char_to_uint(buf);
    };
}

namespace global_commands {//This namespace contains functions and data for altering the GLOG_CMD register
    enum values{//inputting values for setting the register
        fautonull	 =0x0001,  //quickly corrects gyroscope to output all registers to 0 wehn motionless
        factoryrestore	 =0x0002,  //restore default calibration settings from flash memory
        DAClatch	 =0x0004,  //this moves the AUX_DAC register to the DAC input register (see table 21 in notebook)
        flashupdate	 =0x0008,  //this performs a backup of the current register settings into flash memory.
        pautonull	 =0x0010,  //precisely corrects gyrocope to output all registers to 0 when motionless.  Note: takes at least 30 seconds
        softreset	 =0x0080   //stops device, powers off, powers on, reloads registers from flash memory
    };
    
    void gcommand(int cmd,char* buf ){//This function sends the command given in cmd to the GLOB_CMD register
        uint16_t command= write_reg(GLOB_CMD,LOWER_BYTE)|(uint16_t)cmd;  //composing command as uint1
        uint_to_char(command,buf);  //converting command to char*
        bcm2835_spi_transfern(buf, sizeof(buf));  //wrting command
        nanosleep(&stall,NULL);  //stall time for next command
    };
}

namespace data_acq {//this namespace contains functions for calling data
    void read_data(int reg, double* output, int range){ //This function reads data from register reg and stores the data in output.  note, you must input the gyroscope range setting if you are reading the gyroscope.
        using namespace filter_set;
        //composing command
        //uint_to_char(reg,buf);
        char buf[]={(char)((uint16_t)(reg>>8)), (char)((uint16_t)(reg<<8)>>8)};
        
        printf("The input is: %02x %02x. \n",buf[0], buf[1]);
        //sending command
        bcm2835_spi_transfern(buf,sizeof(buf));
        //enforcing minimum stall time
        nanosleep(&stall,NULL);
        //reading response
        bcm2835_spi_transfern(buf,sizeof(buf));
        //stalling for next command
        //nanosleep(&stall,NULL);
        printf("The output is: %02x %02x. \n",buf[0], buf[1]);
        uint16_t out = char_to_uint(buf);
        
        //retrieving individual bits for data interpretation
        bitset<16> q(out);
        //data interpretations
        if (reg==DIAG_STAT){
            if (q[15]==1){
                cout<<"Z-axis acceleromenter self-test failure."<<endl;
            }else if(q[15]==0){
                cout<<"Z-axis accelerometer self-test pass."<<endl;
            };
            
            if (q[14]==1){
                cout<<"Y-axis acceleromenter self-test failure."<<endl;
            }else if(q[14]==0){
                cout<<"Y-axis accelerometer self-test pass."<<endl;
            };
            
            if (q[13]==1){
                cout<<"X-axis acceleromenter self-test failure."<<endl;
            }else if(q[13]==0){
                cout<<"X-axis accelerometer self-test pass."<<endl;
            };
            
            if (q[12]==1){
                cout<<"Z-axis gyroscope self-test failure."<<endl;
            }else if(q[12]==0){
                cout<<"Z-axis gyroscope self-test pass."<<endl;
            };
            
            if (q[11]==1){
                cout<<"Y-axis gyroscope self-test failure."<<endl;
            }else if(q[11]==0){
                cout<<"Y-axis gyroscope self-test pass."<<endl;
            };
            
            if (q[10]==1){
                cout<<"X-axis gyroscope self-test failure."<<endl;
            }else if(q[10]==0){
                cout<<"X-axis gyroscope self-test pass."<<endl;
            };
            
            if (q[9]==1){
                cout<<"Alarm 2 status active."<<endl;
            }else if(q[9]==0){
                cout<<"Alarm 2 status inactive."<<endl;
            };
            
            if (q[8]==1){
                cout<<"Alarm 1 status active."<<endl;
            }else if(q[8]==0){
                cout<<"Alarm 1 status inactive."<<endl;
            };
            
            if (q[6]==1){
                cout<<"Flash test, checksum, failed."<<endl;
            }else if(q[6]==0){
                cout<<"Flash test, checksum, passed."<<endl;
            };
            
            if (q[5]==1){
                cout<<"Self-test diagnostic error fail."<<endl;
            }else if(q[5]==0){
                cout<<"Self-test diagnostic error pass."<<endl;
            };
            
            if (q[4]==1){
                cout<<"Sensor outside range bounds."<<endl;
            }else if(q[4]==0){
                cout<<"Sensor wihtin range bounds."<<endl;
            };
            
            if (q[3]==1){
                cout<<"SPI communication failure."<<endl;
            }else if(q[3]==0){
                cout<<"No SPI communication failure."<<endl;
            };
            
            if (q[2]==1){
                cout<<"Flash update failure."<<endl;
            }else if(q[2]==0){
                cout<<"No flash update failure ."<<endl;
            };
            
            if (q[1]==1){
                cout<<"Power supply above 5.25 volts."<<endl;
            };
            
            if (q[0]==1){
                cout<<"Power supply below 4.75 Volts."<<endl;
            }else if(q[0]==0 && q[1]==0){
                cout<<"Power supply fine."<<endl;
            };
            
        }else{
            if (q[15]==0  && reg!=PROD_ID){
                cout<<"This data has not yet been updated since the last time it was read"<<endl;
            };
            if (q[14]==1 && reg!=PROD_ID){
                cout<<"There is an error.  Check DIAG_STAT to see what it is"<<endl;
            };
            
            switch ((int)reg){
                case SUPPLY_OUT:
                    *output=double(int16_t(out << 2) >> 2)*2.42/1000.0; //Volts
                    break;
               
                case GYRO_OUT:
                    if (range==pm300){
                        *output=double(int16_t(out << 2) >> 2)*0.05;  //deg per sec
                        break;
                    }else if(range==pm150){
                        *output=double(int16_t(out << 2) >> 2)*0.025;  //deg per sec
                        break;
                    }else if(range==pm75){
                        *output=double(int16_t(out << 2) >> 2)*0.0125;  //deg per sec
                        break;
                    }else{
                        cout<<"Undefined range value."<<endl;
                        throw -1;
                        break;
                    };
                
                case XACCL_OUT:
                    *output=double(int16_t(out << 2) >> 2)*10.0/1000.0*9.81;  //m/s^2
                    break;
                case YACCL_OUT:
                    *output=double(int16_t(out << 2) >> 2)*10.0/1000.0*9.81;  //m/s^2
                    break;
                case ZACCL_OUT:
                    *output=double(int16_t(out << 2) >> 2)*10.0/1000.0*9.81;  //m/s^2
                    break;
                
                case TEMP_OUT:
                    *output=25.0+double(int16_t(out << 4) >> 4)*0.14;  //degc
                    break;
                
                case ROLL_OUT:
                    *output=double(int16_t(out << 2) >> 2)*.044; //DEG
                    break;
                case PITCH_OUT:
                    *output=double(int16_t(out << 2) >> 2)*.044; //DEG
                    break;
                
                case AUX_ADC:
                    *output=double(uint16_t(out << 4) >> 4)*0.81/1000.0;  //Volts
                    break;
                case PROD_ID:
                    *output=double(uint16_t(out));  //product id
                    break;
                default:
                    cout<<"invalid register"<<endl;
                    break;
            };
        };
    };
    
    //void burstread
}

namespace callibration {//This namespace contains functions and data for setting the callibration parameters of the imu
    void callibset(int reg, double value) {//This function sets the callibration to the given value for the given register.
        /* gyroscope	 =>value in deg/sec
         * accelerometer	=>value in mg
         * magnetometer HIF	=>value in mgauss
         * magnetometer SIF	=>value is scale factor betweeen 0 and 2*/
        int16_t comval=0;
        if(reg==XACCL_OFF || reg==YACCL_OFF || reg==ZACCL_OFF){
            comval=value/0.6;
        }else if(reg==GYRO_OFF ){
            comval=value/0.0125;
        
        }else{
            cout<<"invalid register, please try again"<<endl;
            comval=0;
            throw -1;
        };
        int16_t cmd1=uint16_t(comval<<8)>>8; //conversioon has been checked
        int16_t cmd2=(uint16_t)comval>>8;  //conversion has been checked.
        uint32_t len=16;
        uint16_t commandl= write_reg(reg,LOWER_BYTE)|cmd1;
        uint16_t commandu= write_reg(reg,UPPER_BYTE)|cmd2;
        
        bcm2835_spi_writenb((char*)(&commandl),len);
        bcm2835_spi_writenb((char*)(&commandu),len);
    }
}

using namespace filter_set;
using namespace data_acq;


int initialize(void){
    if (!bcm2835_init()){
        return -1;
    }
    
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0,LOW);
    return 0;
}

int main( int argc, char **argv){
    
    std::cerr << "Running IMU test" << std::endl;
    //bcm2835_set_debug(1);
    //initializing the spi pins
    int suc=initialize();
    
    //checking to see if the spi pins successfully initialized
    if (suc!=0){
        return -1;
    }
    
    char buf1[] = { 0xBE, 0x00 }; /*setting buff command to restart the imu.  This ensures everything is operational*/
    
    bcm2835_spi_transfern( buf1,sizeof(buf1) ); /*This sends the command to the device and simultaneously reads the output from the last command.  In this case it will be 0 since there was no previous command*/
    
    sleep(1);  /*This command is to give the imu time to powerback up.  It could be shorter.  Right now it's set for 1 second which is more than enough.  If you need it to be shorter use something like nanosleep and look in the manual to see the minimum startup time*/
    
    /*This is the first part that needs changed.  By simply calling GLOB_CMD here we are asking to read the GLOB_CMD register.  This activates burst mode*/
     
    uint16_t reg = GLOB_CMD; //calling register to read
    
    ofstream datastorage ("/tmp/data.txt");  //opening a file to store the data
    // creating y2k epoch
    struct tm y2k;  //creating structure for epoch
    
    y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
    y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;  //setting up epoch time structure
    
    struct timeval timechk;
    
    //system( printf("Hello"));
    
    system("raspivid -n -fps 30 -t 20000 -o /tmp/test.h264 &");
    
    //fprintf("Line after camera command");
    
    if (datastorage.is_open()){
    
        for( size_t i=0 ; i<2000; i++ ){/*It is best practice to define variables in the scope in which they will be used.  It can cause major headaches if you don't*/
            
            //sleep(1);
            
            ///////////////////////////////////////////////////Time Stamp Stuff////////////////////////////////////////
            time_t timer;  //creating timer variable
            
            time(&timer);  /* get current time - move this to whenever you want to record the time */
            
            double timestamp = difftime(timer,mktime(&y2k)); //calculating time since y2k epoch
            
            gettimeofday(&timechk , NULL);
            
            double timestamp2 = (double)timechk.tv_sec + (double)timechk.tv_usec/1000000.0;
            
            /////////////////////////////////////////////////Initialize Burst Mode///////////////////////////////////
            char init_burst_buf[] = {(char)((uint16_t)(reg>>8)), (char)((uint16_t)(reg<<8)>>8)}; /*This is what we're going to use to initialize the burst mode*/
            
            bcm2835_spi_transfern( init_burst_buf, sizeof(init_burst_buf) ); /*This is sending the burst mode command*/
            
            nanosleep( &burst_stall , NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            //////////////////////////////////////////////////Data Acq/////////////////////////////////////
            
            //////////////////////////////////////////////////SUPPLY//////////////////////////////////////////////
            
            char supply_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the supply register.*/
            
            bcm2835_spi_transfern( supply_buf, sizeof(supply_buf) ); /* Reading the supply_buf output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            //////////////////////////////////////////////////GYROSCOPE//////////////////////////////////////////////
            
            char gyro_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the xgyro register.*/
            
            bcm2835_spi_transfern( gyro_buf, sizeof(gyro_buf) ); /* Reading the xgyro output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            //char ygyro_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the ygyro register.*/
            
            //bcm2835_spi_transfern( ygyro_buf, sizeof(ygyro_buf) ); /* Reading the ygyro output */
            
            //nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            //char zgyro_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the zgyro register.*/
            
            //bcm2835_spi_transfern( zgyro_buf, sizeof(zgyro_buf) ); /* Reading the zgyro output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            /////////////////////////////////////////////////ACCELEROMETER/////////////////////////////////////////////
            
            char xaccl_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the xaccl register.*/
            
            bcm2835_spi_transfern( xaccl_buf, sizeof(xaccl_buf) ); /* Reading the xaccl output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            char yaccl_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the yaccl register.*/
            
            bcm2835_spi_transfern( yaccl_buf, sizeof(yaccl_buf) ); /* Reading the yaccl output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            char zaccl_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the zaccl register.*/
            
            bcm2835_spi_transfern( zaccl_buf, sizeof(zaccl_buf) ); /* Reading the zaccl output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            
            /////////////////////////////////////////////////TEMPERATURE////////////////////////////////////////////////
            
            char temp_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the temp register.*/
            
            bcm2835_spi_transfern( temp_buf, sizeof(temp_buf) ); /* Reading the temp output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            /////////////////////////////////////////////////INCLINOMETER////////////////////////////////////////////////
            
            char pitch_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the temp register.*/
            
            bcm2835_spi_transfern( pitch_buf, sizeof(pitch_buf) ); /* Reading the pitch output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            char roll_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the temp register.*/
            
            bcm2835_spi_transfern( roll_buf, sizeof(roll_buf) ); /* Reading the roll output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is coded above*/
            
            ////////////////////////////////////AUXILIARY ANALOGUE TO DIGITAL CONVERTER VOLTAGE//////////////////////////
            
            char aux_buf[]={(0x00), (0x00)}; /*This will be the buffer where we recieve the info from the aux register.*/
            
            bcm2835_spi_transfern( aux_buf, sizeof(aux_buf) ); /* Reading the aux output */
            
            nanosleep( &burst_stall, NULL ); /*This command enforces the stall time for burst mode.  It is probably not needed here because hte following code will probably take this long anyway, but just to be sure we will execute it*/
            
            ///////////////////////////////////////////////////DATA INTERPRETATION/////////////////////////////////////////
            
            /////////////////////////////////////////////////////SUPPLY VOLTAGE/////////////////////////////////////////////
            
            uint16_t sup_temp = ( (uint16_t) supply_buf[0] << 8 )+(uint16_t) supply_buf[1];  /*recomposing the supply voltage output using bit shifts.*/
            
            uint16_t supply_output = ((uint16_t)(sup_temp<<2)>>2); /*here we are interpretting the supply output into LSB form.  It is an unsigned output hence the uint16_t cast*/
            
            double supply_voltage=(double)supply_output * 2.418/1000.0;  /*here we are applying the scale and casting to double to get the readable output*/
            
            
            /////////////////////////////////////////////////GYROSCOPE OUTPUT/////////////////////////////////////////////
            
            uint16_t gyro_temp = ( (uint16_t) gyro_buf[0] << 8 )+(uint16_t) gyro_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t gyro_output = ((int16_t)(gyro_temp<<2)>>2); /*here we are interpretting the output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double gyro=(double)gyro_output * 0.05;  /*here we are applying the scale and casting to double to get the readable output*/
            
            
            ////////////////////////////////////////////////ACCELEROMETER OUTPUT///////////////////////////////////////////////
            
            uint16_t xaccl_temp = ( (uint16_t) xaccl_buf[0] << 8 )+(uint16_t) xaccl_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t xaccl_output = ((int16_t)(xaccl_temp<<2)>>2); /*here we are interpretting the output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double xaccl=(double)xaccl_output * 3.33/1000.0;  /*here we are applying the scale and casting to double to get the readable output*/
            
            uint16_t yaccl_temp = ( (uint16_t) yaccl_buf[0] << 8 )+(uint16_t) yaccl_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t yaccl_output = ((int16_t)(yaccl_temp<<2)>>2); /*here we are interpretting the supply output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double yaccl=(double)yaccl_output * 3.33/1000.0;  /*here we are applying the scale and casting to double to get the readable output*/
            
            uint16_t zaccl_temp = ( (uint16_t) zaccl_buf[0] << 8 )+(uint16_t) zaccl_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t zaccl_output = ((int16_t)(zaccl_temp<<2)>>2); /*here we are interpretting the supply output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double zaccl=(double)zaccl_output * 3.33/1000.0;  /*here we are applying the scale and casting to double to get the readable output*/
            
            
            
            ////////////////////////////////////////////////TEMPERATURE OUTPUT///////////////////////////////////////////////

            uint16_t temp_temp = ( (uint16_t) temp_buf[0] << 8 )+(uint16_t) temp_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t temp_output = ((int16_t)(temp_temp<<4)>>4); /*here we are interpretting the supply output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double temperature=(double)temp_output * 0.14;  /*here we are applying the scale and casting to double to get the readable output*/
            
            ////////////////////////////////////////////////inclinometer OUTPUT///////////////////////////////////////////////

            uint16_t pitch_temp = ( (uint16_t) pitch_buf[0] << 8 )+(uint16_t) pitch_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t pitch_output = ((int16_t)(pitch_temp<<2)>>2); /*here we are interpretting the supply output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double pitch=(double)pitch_output * .004;  /*here we are applying the scale and casting to double to get the readable output*/
            
            uint16_t roll_temp = ( (uint16_t) roll_buf[0] << 8 )+(uint16_t) roll_buf[1];  /*recomposing the output using bit shifts.*/
            
            int16_t roll_output = ((int16_t)(roll_temp<<2)>>2); /*here we are interpretting the supply output into LSB form.  It is a signed output hence the int16_t cast*/
            
            double roll=(double)roll_output * .004;  /*here we are applying the scale and casting to double to get the readable output*/
            
            
            ///////////////////////////////////////////////AUXILLIARY ADC OUTPUT//////////////////////////////////////////////
            
            uint16_t aux_temp = ( (uint16_t) aux_buf[0] << 8 )+(uint16_t) aux_buf[1];  /*recomposing the output using bit shifts.*/
            
            uint16_t aux_output = ((uint16_t)(aux_temp<<4)>>4); /*here we are interpretting the supply output into LSB form.  It is an unsigned output hence the uint16_t cast*/
            
            double aux_voltage=(double)aux_output * 806.0/(double)1e6;  /*here we are applying the scale and casting to double to get the readable output*/

            ////////////////////////////////////////////////DATA STORAGE///////////////////////////////////////////////////////
            
            datastorage << std::setprecision(16) <<timestamp2 <<  "    " << supply_voltage << "," << gyro << "," << xaccl << "," << yaccl << "," << zaccl << "," << temperature << "," << roll << "," << pitch <<  "," << aux_voltage << "\n";
            
        }
        
        datastorage.close(); //closing the data storage file
        
    }else{
        cerr<<"The file tried to open.  Please try again."<<endl;
    }
    return 0;
}


