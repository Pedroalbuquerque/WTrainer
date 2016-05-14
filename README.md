#Wtrainer


Trainer stands for Wireless Trainer  .

In order to help a new pilot to fly an RC model, two RC radios can be connected so that the trainer can help the noob in an emergency by taking control and giving back after recovery.

This traditionally requires a cable.

this Projects implements the “cable” wirelessly .

It converts the input from noob radio to a radio wave using the WTrainer_in code and then converts back the radio waves to a signa to be output to the trainer radio.

The <B>Wtrainer_in</B> sets a default value for each of the 8 channels (configurable on code), and when a new value is received form the input pin it is stored and transmitted.

The <B>WTrainer_out</B> assigns a default value to the 8 channels and listens to the incoming radio info to assign to each channel.

If radio info takes to long to arrive, a failsafe mode is triggered and channel are assigned once more a failsafe value and the value is output to the RC radio.

