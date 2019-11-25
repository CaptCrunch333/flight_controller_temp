#include "ProcessVariableReference.hpp"

ProcessVariableReference::ProcessVariableReference(std::string t_name, block_type t_type) : Reference(t_name, t_type) {
    
    _reference_type = reference_type::process_variable_ref;
}

ProcessVariableReference::~ProcessVariableReference() {

}

reference_type ProcessVariableReference::getReferenceType(){
    return _reference_type;
}

DataMessage* ProcessVariableReference::receive_msg_internal(DataMessage* t_msg){
    std::cout << " I AM HEREEEEEEE RECEIVE MSG FROM REFERENCE" << std::endl;
}

DataMessage* ProcessVariableReference::receive_msg_internal(){
    
}