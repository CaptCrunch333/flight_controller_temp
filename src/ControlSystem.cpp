#include "ControlSystem.hpp"
#include "../include/Switcher.hpp"

ControlSystem::ControlSystem() {
    // this->add_callback_msg_receiver((msg_receiver*)controllerSwitcher);
    // this->add_callback_msg_receiver((msg_receiver*)referenceSwitcher);
    // this->add_callback_msg_receiver((msg_receiver*)providerSwitcher);
    // providerSwitcher->add_callback_msg_receiver((msg_receiver*)referenceSwitcher);
    // referenceSwitcher->add_callback_msg_receiver((msg_receiver*)controllerSwitcher);
    // controllerSwitcher->add_callback_msg_receiver((msg_receiver*)this);
    
}

ControlSystem::ControlSystem(control_system t_control_system) {
    _control_system = t_control_system;
    this->add_callback_msg_receiver((msg_receiver*)controllerSwitcher);
    this->add_callback_msg_receiver((msg_receiver*)referenceSwitcher);
    this->add_callback_msg_receiver((msg_receiver*)providerSwitcher);
    providerSwitcher->add_callback_msg_receiver((msg_receiver*)referenceSwitcher);
    referenceSwitcher->add_callback_msg_receiver((msg_receiver*)controllerSwitcher);
    controllerSwitcher->add_callback_msg_receiver((msg_receiver*)this);
}

ControlSystem::~ControlSystem() {

}

void ControlSystem::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* control_msg = (ControlSystemMessage*)t_msg;

    }

}

void ControlSystem::getStatus(){
    
    for(Switcher* s : _switchers){
        if(s->getActiveBlock() != nullptr){
            std::cout << "For Control System " << static_cast<int>(_control_system) << std::endl;
            std::cout << "For switcher " << s->getName() << " the active block is " << s->getActiveBlock()->getName() << std::endl;
        }     
    }
}

Switcher* ControlSystem::getControllerSwitcher(){
    return controllerSwitcher;
}

Switcher* ControlSystem::getReferenceSwitcher(){
    return referenceSwitcher;
}

Switcher* ControlSystem::getProviderSwitcher(){
    return providerSwitcher;
}

void ControlSystem::switchBlock(Block* t_from, Block* t_to){
    ControlSystemMessage* switch_msg = new ControlSystemMessage(control_system_msg_type::switch_in_out, t_from, t_to);
    
    this->emit_message((DataMessage*)switch_msg);
}

void ControlSystem::addBlock(Block* t_block){
    ControlSystemMessage* add_block_msg = new ControlSystemMessage(control_system_msg_type::add_block, t_block);

    this->emit_message((DataMessage*)add_block_msg);

}

void ControlSystem::changePIDSettings(PID_parameters* t_pid_para){ //TODO refactor through receive_msg, a remote msg should change the pid

    ControlSystemMessage* change_PID_msg = new ControlSystemMessage(control_system_msg_type::change_PID_settings, t_pid_para);

    this->emit_message((DataMessage*)change_PID_msg);

}