#include "MSP.h"

MSP::MSP(char* port, int baud)
{
	// initialise serial port
    sp = serialOpen(port, baud);
    cout << "MSP communication established" << endl;

}

void MSP::send_msg(uint8_t CMD, uint16_t idata[], uint8_t n_bytes, char mode)
{
	uint8_t checksum = 0;
    uint8_t data[n_bytes];

    encode_data(idata, data, n_bytes);

    // send header
    serialPutchar(sp, '$');
    serialPutchar(sp, 'M');
    serialPutchar(sp, '<');
    serialPutchar(sp, n_bytes);
    serialPutchar(sp, CMD);

	checksum ^= n_bytes;
	checksum ^= CMD;

    // if write mode, send data
	if (mode == 'w')
    {
        checksum = mspSerialChecksumBuf(checksum, data, n_bytes);

        int i = 0;
        while (i < n_bytes)
        {
            //cout << (int)data[i] << ' ';
            serialPutchar(sp, data[i++]);
        }

        serialPutchar(sp, checksum);

        //cout << (int) checksum << endl;

    }
    else
    {
        serialPutchar(sp, checksum);
    }

} // END send message


void MSP::read_msg()
{
    while (!serialDataAvail(sp)) {}

    while (serialDataAvail(sp))
    {
        uint8_t c = serialGetchar(sp);
        //cout << (int)c << ' ';

        switch(c_state)
        {
            default:
            case IDLE:
                c_state = (c=='$') ? HEADER_M:IDLE;
                break;

            case HEADER_M:
                c_state = (c=='M') ? HEADER_ARROW:IDLE;
                break;

            case HEADER_ARROW:
                c_state = (c=='>') ? HEADER_SIZE : IDLE;
                break;

            case HEADER_SIZE:
                if (c > DATA_SIZE)  c_state = IDLE;
                else
                {
                    msg.data_length = c;
                    msg.checksum = 0;
                    msg.idx = 0;
                    msg.checksum ^= c;

                    c_state = HEADER_CMD;
                }
                break;

            case HEADER_CMD:
                msg.CMD = c;
                msg.checksum ^= c;

                c_state = HEADER_DATA;
                break;

            case HEADER_DATA:
                if (msg.idx < msg.data_length)
                {
                    msg.inBuf[msg.idx++] = c;
                    msg.checksum ^= c;
                }
                else // msg.idx >= msg.data_length
                {

                    if (c==msg.checksum)
                        c_state = MESSAGE_RECEIVED;
                    else
                        c_state = IDLE;
                }
                break;


        } // END c_state

    } // END DATA AVAILABLE

    decode_msg();

} // END read message


void MSP::decode_msg()
{

    switch(msg.CMD)
    {
        default:
        case MSP_STATUS:

            break;

        case MSP_ALTITUDE:

            break;

        case MSP_MOTOR:
            motor.m1 = uI8_toUINT16(msg.inBuf,0);
            motor.m2 = uI8_toUINT16(msg.inBuf,1);
            motor.m3 = uI8_toUINT16(msg.inBuf,2);
            motor.m4 = uI8_toUINT16(msg.inBuf,3);
            break;

        case MSP_ATTITUDE:

            attitude.pitch = uI8_toINT16(msg.inBuf, 1)/10.f;
            attitude.roll = uI8_toINT16(msg.inBuf, 0)/10.f;
            attitude.yaw = uI8_toINT16(msg.inBuf, 2);
            break;

        case MSP_RC:
            rcIn.ch1 = uI8_toUINT16(msg.inBuf,0);
            rcIn.ch2 = uI8_toUINT16(msg.inBuf,1);
            rcIn.ch3 = uI8_toUINT16(msg.inBuf,2);
            rcIn.ch4 = uI8_toUINT16(msg.inBuf,3);
            rcIn.ch5 = uI8_toUINT16(msg.inBuf,4);
            rcIn.ch6 = uI8_toUINT16(msg.inBuf,5);
            rcIn.ch7 = uI8_toUINT16(msg.inBuf,6);
            rcIn.ch8 = uI8_toUINT16(msg.inBuf,7);
            break;

    } // END switch m


} // END decode message

void MSP::encode_data(uint16_t idata[], uint8_t *odata, int len)
{

    // from 16 bits array to 8 bits array
    for (int i = 0; i<len/2 ; i++)
    {
        *(odata + i*2)      = idata[i];

        *(odata + i*2 + 1)  = idata[i] >> 8;

        //cout << (int) *(odata+i*2) << ' ' << (int) *(odata+i*2+1) << ' ' ;
    }


    //cout << endl;
}


//----------------------------------------------//
//-------- Get RC channels from receiver -------//
//----------------------------------------------//
void MSP::get_rx_CH(int data[])
{
    if (!((data[0] == 1100 || data[0] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch1 = data[0];    // roll
    if (!((data[1] == 1100 || data[1] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch2 = data[1];    // pitch
    if (!((data[2] == 1100 || data[2] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch3 = data[2];    // yaw
    if (!((data[3] == 1100 || data[3] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch4 = data[3];    // throttle
    if (!((data[5] == 1100 || data[5] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch6 = data[5];
    if (!((data[6] == 1100 || data[6] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch7 = data[6];
    if (!((data[7] == 1100 || data[7] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))     this->rx.ch8 = data[7];

    if (!((data[0] == 1100 || data[0] == 1900 || data[1] == 1100 || data[1] == 1900 ||
           data[2] == 1100 || data[2] == 1900 || data[3] == 1100 || data[3] == 1900) &&
                              data[4] == 1900 && this->rx.ch4 == 1100))
    this->rx.ch5 = data[4];

    if (!((data[0] == 1100 || data[0] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[0] = data[0]; // roll
    if (!((data[1] == 1100 || data[1] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[1] = data[1]; // pitch
    if (!((data[2] == 1100 || data[2] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[2] = data[2]; // yaw
    if (!((data[3] == 1100 || data[3] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[3] = data[3]; // throttle
    if (!((data[5] == 1100 || data[5] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[5] = data[5];
    if (!((data[6] == 1100 || data[6] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[6] = data[6];
    if (!((data[7] == 1100 || data[7] == 1900) && data[4] == 1900 && this->rx.ch4 == 1100))    this->msp_rc_data[7] = data[7];

    if (!((data[0] == 1100 || data[0] == 1900 || data[1] == 1100 || data[1] == 1900 ||
           data[2] == 1100 || data[2] == 1900 || data[3] == 1100 || data[3] == 1900) &&
                              data[4] == 1900 && this->msp_rc_data[4] == 1100))
    this->msp_rc_data[4] = data[4];

}


// -----------------------------------------------------------------------------------------
// ------------------------  CLASS helper functions ----------------------------------------
// -----------------------------------------------------------------------------------------
uint8_t MSP::mspSerialChecksum(uint8_t checksum, uint8_t byte)
{
	return checksum ^ byte;
}

uint8_t MSP::mspSerialChecksumBuf(uint8_t checksum, uint8_t *data, int len)
{
	while (len-- > 0) {
		checksum = mspSerialChecksum(checksum, *data++);
		//cout << "checksum " << (int)checksum << endl;
	}

	return checksum;
}


// -----------------------------------------------------------------------------------------
// ------------------------  OTHER helper functions ----------------------------------------
// -----------------------------------------------------------------------------------------
int16_t uI8_toINT16(uint8_t buf[], int idx)
{
    return (buf[idx*2+1] << 8) | buf[idx * 2];
}

uint16_t uI8_toUINT16(uint8_t buf[], int idx)
{
    return (buf[idx*2+1] << 8) | buf[idx * 2];
}

