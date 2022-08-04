clear; clc;

%% Input section

%button pins
Accelerator_button  = 'A1';  %analog pin A1
Brake_button        = 'A2';  %analog pin A2
CC_ON_button        = 'A3';  %analog pin A3
ACC_button          = 'A4';  %analog pin A4
CC_OFF_button       = 'A5';  %analog pin A5
CC_LED_indicator    = 'D13'; %digital pin 13

%Ultrasonic sensor pins
sonar_trigger       = 'D9';  %digital pin 9
sonar_echo          = 'D8';  %digital pin 8

%setting up Arduino, LCD, and ultrasonic sensor objects
due = arduino('com4','uno','libraries',{'I2C', 'SPI', 'Ultrasonic','ExampleLCD/LCDAddon'},'ForceBuildOn',true);

%configuring all the pins of arduino
configurePin(due,Accelerator_button,'Pullup');      %setting up accelerator button.
configurePin(due,Brake_button,'Pullup');            %setting up brake button.
configurePin(due,CC_ON_button,'Pullup');            %setting up cruise control button.
configurePin(due,CC_OFF_button,'Pullup');           %setting up cancel button.
configurePin(due,ACC_button,'Pullup');              %setting up adaptive button
configurePin(due,CC_LED_indicator,'DigitalOutput'); %setting up LED light.

%% defining variables

current_speed     = 0;    %starting speed of the vehicle is 0 mph.
delay_button      = 0.1;  %pause while pressing accelerator or brake button (sec).
delay_decelration = 1;    %pause while natural decelration (sec).
delay_acc         = 1.5;  %pause while adaptive mode is ON (sec).
decay             = 0;    %initial decay of speed in adaptive mode (sec).
blink_rate        = 0.2;  %rate of blinking when the adaptive mode is ON.

start_time        = 0;   %this is the start of the main program time
end_time          = 0;   %this is the total elapsed time until the button is pressed

sensor_range      = 0.2; %if object is in this range, only then speed is decreased.

%boolean variables
is_cc_on          = false; %initially cruise control mode is off
is_acc_on         = false; %initially Adaptive cruise control mode is off

%% main Program begins

writeDigitalPin(due,CC_LED_indicator,0); %the cruise control LED is OFF.

%creating LCD display object - myDisplay
myDisplay = addon(due,'ExampleLCD/LCDAddon','RegisterSelectPin','D7','EnablePin','D6','DataPins',{'D5','D4','D3','D2'});
initializeLCD(myDisplay);    %Initialize the LCD
clearLCD(myDisplay);         %This clears the LCD

%executing displySpeed function: default
displaySpeed(myDisplay,'Speed: ', current_speed);

%creating utrasonic sensor object - sonar
sonar = ultrasonic(due, sonar_trigger, sonar_echo); 

while true
    
    %checks whether the accelerator button is pressed
    Accelerator_button_status = readDigitalPin(due,Accelerator_button);
    
    %if accelerator button is pressed, then this if block will get executed
    if Accelerator_button_status == 1
        current_speed = current_speed+1;          %increase the current speed by 1.
        displaySpeed(myDisplay,'Speed: ', current_speed);  %diplaying speed on LCD.
        end_time = 0;                                      %resetting the end time to 0.
        pause(delay_button);                               %wait for 0.1 sec.
    end
    
    %checks whether the brake button is pressed
    Brake_button_status = readDigitalPin(due,Brake_button);
    
    if (Brake_button_status == 1) && (current_speed>0)
        current_speed = current_speed-1;          %decresing the current speed by 1.
        displaySpeed(myDisplay,'Speed: ', current_speed);  %displaying current speed.
        end_time = 0;                                      %resetting the end time to 0.
        pause(delay_button);                               %wait for 0.1 sec.
    end
    
    %when no button is pressed until about 2 sec, this if block will get executed.
    if (current_speed>0) && (Accelerator_button_status == 0) && (end_time>5) 
        current_speed = current_speed-1;          %decreasing the current speed by 1
        displaySpeed(myDisplay,'Speed: ', current_speed);  %displaying the current speed.
        end_time = 0;                                      %resetting the end time to 0.
        pause(delay_decelration);                          %wait for 0.1 sec.
    end
    
    %% Cruise Control functionality
    
    % first checks the status of the cruise control button.
    CC_ON_button_status = readDigitalPin(due,CC_ON_button);
    
    %once the cruise control mode is active:
    while (CC_ON_button_status == 1) || (is_cc_on)
        
        is_cc_on = true;      %ensures that cruise control mode is turned on.
        
        %turn ON the LED, indicating the cruise control mode is on.
        writeDigitalPin(due,CC_LED_indicator,1);
        
        %cancel cruise control mode functionality:
        %checks the status of the cancel button
        CC_OFF_button_status = readDigitalPin(due,CC_OFF_button);
        
        if (CC_OFF_button_status == 1) && ( is_cc_on || is_acc_on)
            
            is_cc_on = false;               %turn of cruise control.
            is_acc_on = false;              %turn of adaptive cruise control. 
            printLCD(myDisplay, 'CC OFF');  %diplay cruise control OFF.
            
            %turn OFF the LED, indicating that cruise conrol is OFF
            writeDigitalPin(due,CC_LED_indicator,0);
            break; %terminate the loop and control will be handed over to the main program.
            
        end
        
        if ~is_acc_on %this block will only be executed if adaptive cruise control mode is off
            
            %checks if the accelerator button is pressed in the cruise control mode
            Accelerator_button_status = readDigitalPin(due,Accelerator_button);
            
            %if acceleration button is pressed, then this block will be executed.
            if Accelerator_button_status == 1
                current_speed = current_speed+1;  %incearsing the speed by 1.
                pause(delay_button);                %wait for 0.1 sec
            end
            
            %checks if the brake button is pressed in the cruise control mode
            Brake_button_status = readDigitalPin(due,Brake_button);
            
            %if brake button is pressed, then this block will be executed.
            if (Brake_button_status == 1) && (current_speed>0)
                current_speed = current_speed+1;  %decreasing the speed by 1.
                pause(delay_button);                %wait for 0.1 sec 
            end
            
            set_speed = current_speed;              %set speed for the adaptive mode
            
            %diplay the locked speed.
            displaySpeed(myDisplay, 'CC ON: ', current_speed); 
            
        end
        
        %% Adaptive functionality of the cruise control
        % checks the status of the Adaptive cruise control button
        ACC_button_status = readDigitalPin(due,ACC_button);
        
        if (is_acc_on) || (ACC_button_status == 1)
            
            %ensures that adaptive mode is on even the button is not pressed.
            is_acc_on = true;
            
            object_distance = readDistance(sonar);  %measures the distace of the object ahead.
            
            %if the distance is in the range, then this block will get executed.
            if (object_distance < sensor_range) && (current_speed > 0)
                
                %exponentially decreasing the speed
                current_speed = current_speed - round(exp(0.15*decay));
                pause(delay_acc);                   %wait for the 1.5 sec.
                decay = decay+1;                    %each iteration adds 1 to decay
            
            %if the ditance is outside the range, then this bolck will get executed.
            elseif (object_distance> sensor_range) && (current_speed < set_speed)
                current_speed = current_speed+1;  %increase speed by 1
                pause(delay_acc);                   %wait for 1.5 sec.
                decay = 0;                          %resetting decay to 0.   

            end 
            
            %blinks the LCD display while Adaptive cruise control is ON.
            %blinking rate is set to 0.5 sec.
            
            writeDigitalPin(due,CC_LED_indicator,0);
            clearLCD(myDisplay);
            pause(blink_rate); 
            writeDigitalPin(due,CC_LED_indicator,1);
            displaySpeed(myDisplay, 'ACC ON: ', current_speed);
            pause(blink_rate);
       
        end
        
    end
    
    end_time = end_time + 0.1;  %each iteration adds 0.1 to the last end time.
    
    if end_time > 100
        clearLCD(myDisplay);
        break;
    end
    
end

%% function section:

function displaySpeed(dispObj, message, speed)
 %displaySpeed function
 %displays the message on the LCD.
    printLCD(dispObj, [message, num2str(speed), ' mph'])
end


function result = increment(speed)
%increment function
    result = speed + 1;
end


function result = decrement(speed)
%decrement function
    result = speed -1;
end