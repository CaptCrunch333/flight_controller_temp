#pragma once
#include "Block.hpp"
#include <math.h>
#include "ControllerMessage.hpp"
#include "PID_parameters.hpp"
#define dt_manual 0.01f

class PIDController : public Block{

    private:
        string _name; 
        block_type _type;
        //Chehadeh's code
        PID_parameters parameters;
        bool i_term, d_term, dd_term; //Comparing against booleans is faster
	    bool en_pv_derivation = true, en_anti_windup = false;
        void set_internal_sw(PID_parameters pid_para_x);
        //---------------
        
    public:
        //Chehadeh's code
        float prev_err = 0, prev2_err = 0, prev_pv_rate = 0, accum_u = 0, accum_I = 0;
        void initialize(void*);
        float pid_inc(float err, float pv_first,float pv_second=-1);
        float pid_direct(float err, float pv_first,float pv_second=-1);
        void set_I_term(float);
        //---------------
        void receive_msg_data(DataMessage* t_msg);
        void switchIn(DataMessage*);
        DataMessage* switchOut();
        string getName();
        block_type getType();

        PIDController(string name, block_type type);
        ~PIDController();
};