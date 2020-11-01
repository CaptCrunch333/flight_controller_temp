#include "ROSUnit_ResetController.hpp"
ROSUnit_ResetController* ROSUnit_ResetController::_instance_ptr = NULL;
IntegerMsg ROSUnit_ResetController::_reset_msg;
Port* ROSUnit_ResetController::_output_port = new OutputPort(ports_id::OP_0_DATA, NULL);

ROSUnit_ResetController::ROSUnit_ResetController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_reset_controller = t_main_handler.advertiseService("reset_controller", callbackResetController);
    _instance_ptr = this;
    _ports = {_output_port};
}   

ROSUnit_ResetController::~ROSUnit_ResetController() {

}

bool ROSUnit_ResetController::callbackResetController(flight_controller::Reset_Controller::Request &req, flight_controller::Reset_Controller::Response &res){

    _reset_msg.data = req.id;;

    _output_port->receiveMsgData(&_reset_msg);

    return true;
}