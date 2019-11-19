#pragma once
#include "Switcher.hpp"
#include "msg_emitter.hpp"
#include "msg_receiver.hpp"
#include <vector>
#include "SwitchMessage.hpp"

class ControlSystem : public msg_emitter, public msg_receiver{

    private:
        Switcher* controllerSwitcher;
        Switcher* referenceSwitcher;
        Switcher* providerSwitcher;
        std::vector<Switcher*> _switchers{controllerSwitcher, referenceSwitcher, providerSwitcher};

    public:
        void receive_msg_data(DataMessage* t_msg);
        void getStatus();
        void switchAtControllerBlock(Block* t_from, Block* t_to);
        void switchAtReferenceBlock(Block* t_from, Block* t_to);
        void switchAtProviderBlock(Block* t_from, Block* t_to);

        Switcher* getControllerSwitcher();
        Switcher* getReferenceSwitcher();
        Switcher* getProviderSwitcher();

        ControlSystem();
        ~ControlSystem();

    
};