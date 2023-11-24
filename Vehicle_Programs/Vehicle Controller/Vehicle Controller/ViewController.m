//
//  ViewController.m
//  Vehicle Controller
//
//  Created by Nathan Fikes and last edited on 11/22/23.
//
//  Description: IPhone 13 Application on IOS 17.0.1 that allows the device to control the vehicle in my project. This application is designed to talk to a ESP32-WROVER-E peripheral as a delegate. Used to send various commands to the vehicle. These commands can be found at the bottom of the program.

@import CoreBluetooth;
#import "ViewController.h"

#define COMMAND_DELAY_MULT 5
#define MIN_COMMAND_SEND_PERIOD 0.032 * COMMAND_DELAY_MULT

@interface ViewController ()

// Text
@property (weak, nonatomic) IBOutlet UILabel *statusLabel;
@property (weak, nonatomic) IBOutlet UITextField *inputTextField;

// Sliders
@property (weak, nonatomic) IBOutlet UISlider *motorPowerSlider;
@property (weak, nonatomic) IBOutlet UISlider *motorSteeringSlider;
@property (strong, nonatomic) NSTimer *sliderTimerPower;
@property (strong, nonatomic) NSTimer *sliderTimerSteer;

// Toggles
@property (weak, nonatomic) IBOutlet UISwitch *toggleSignalLeft;
@property (weak, nonatomic) IBOutlet UISwitch *toggleSignalRight;
@property (weak, nonatomic) IBOutlet UISwitch *toggleManual;
@property (weak, nonatomic) IBOutlet UISwitch *toggleAuto;

// BLE related
@property (strong, nonatomic) CBCentralManager *centralManager;
@property (strong, nonatomic) CBUUID *serviceUUID;
@property (strong, nonatomic) CBUUID *characteristicUUID;
@property (strong, nonatomic) CBPeripheral *peripheral;
@property (strong, nonatomic) CBCharacteristic *characteristic;

@property (strong, nonatomic) NSDate *lastCommandSendTime;

@end

@implementation ViewController

// Once loaded we can set up a plethora of functions.
- (void)viewDidLoad {
    [super viewDidLoad];
    
    [self setupKeyboardDismissRecognizer];
    
    // Events for the steering slider.
    [self.motorSteeringSlider addTarget:self action:@selector(startSteeringSliderTimer) forControlEvents:UIControlEventTouchUpInside];
    [self.motorSteeringSlider addTarget:self action:@selector(startSteeringSliderTimer) forControlEvents:UIControlEventTouchUpOutside];
    [self.motorSteeringSlider addTarget:self action:@selector(stopSliderTimer:) forControlEvents:UIControlEventTouchDown];
    
    // Events for the power slider.
    [self.motorPowerSlider addTarget:self action:@selector(startPowerSliderTimer) forControlEvents:UIControlEventTouchUpInside];
    [self.motorPowerSlider addTarget:self action:@selector(startPowerSliderTimer) forControlEvents:UIControlEventTouchUpOutside];
    [self.motorPowerSlider addTarget:self action:@selector(stopSliderTimer:) forControlEvents:UIControlEventTouchDown];
    
    // Rotates the sliders by -90 degrees or pi/2 so they are vertical on screen. Uses some Gaffine Transformations.
    self.motorPowerSlider.transform = CGAffineTransformMakeRotation(-M_PI_2);
    self.motorSteeringSlider.transform = CGAffineTransformMakeRotation(-M_PI_2);
    
    // Service UUID and Characteristic UUID unique to peripheral device in this case the ESP32-WROVER-E.
    self.serviceUUID = [CBUUID UUIDWithString:@"4fafc201-1fb5-459e-8fcc-c5c9c331915c"];
    self.characteristicUUID = [CBUUID UUIDWithString:@"beb5483e-36e1-4688-b7f5-ea07361b26a8"];
    
    // Start connecting once everything is all nice and set up.
    [self connect];
}

// We can detect when we are not on the keyboard and then if that is true we can get rid of the keyboard.
- (void)setupKeyboardDismissRecognizer {
    UITapGestureRecognizer *tapRecognizer = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(dismissKeyboard)];
    [self.view addGestureRecognizer:tapRecognizer];
}

// Will get rid of the keyboard, this is useful for when the keyboard covers up other things on the screen.
- (void)dismissKeyboard {
    [self.view endEditing:YES];
}

// Updates the status label with a string passed into the function.
- (void)setStatus:(NSString*)status {
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
        self.statusLabel.text = status;
    });
}

// Starts connecting by updating the status label. If the device is somehow not the central manager then will allow itself to be that way, if it is then we can start the scanning process.
- (void)connect {
    [self setStatus:@"Connecting..."];
    if (!self.centralManager) {
        self.centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:nil options:nil];
    } else if (self.centralManager.state == CBManagerStatePoweredOn) {
        [self startScanning];
    }
}

// Immediately start scanning when the app is run, this is so no buttons are needed. Everything is automatic.
- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    if (central.state == CBManagerStatePoweredOn) {
        [self startScanning];
    }
}

// Scans for the specific peripheral with the service UUID unique to that peripheral, in this case it is the ESP32-WROVER-E that has the matching UUID.
- (void)startScanning {
    [self setStatus:@"Scanning..."];
    [self.centralManager scanForPeripheralsWithServices:@[self.serviceUUID] options:nil];
}

// If there is a disconnect, will stop scanning and connect again.
- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary<NSString *,id> *)advertisementData RSSI:(NSNumber *)RSSI {
    if (!self.peripheral || self.peripheral.state == CBPeripheralStateDisconnected) {
        [self.centralManager stopScan];
        self.peripheral = peripheral;
        [self.centralManager connectPeripheral:peripheral options:nil];
    }
}

// Once connected, declares itself as the peripheral delegate. Also allows the class to interact with the peripheral.
- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral {
    self.peripheral = peripheral;
    peripheral.delegate = self;
    [peripheral discoverServices:@[self.serviceUUID]];
}

// Updates the status label if the peripheral has disconnected. Will automatically try to connect again if this happens.
- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    [self setStatus:@"Disconnected"];
    self.peripheral = nil;
    [self connect];
}

// Discover the services and if failed set the status label to let the user know that the service discovery failed.
- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error {
    if (error) {
        [self setStatus:@"Error discovering services"];
        return;
    }

    for (CBService *service in peripheral.services) {
        if ([service.UUID isEqual:self.serviceUUID]) {
            [peripheral discoverCharacteristics:@[self.characteristicUUID] forService:service];
        }
    }
}

// Discover the characteristic and if failed let the user know that the discovering characteristic stage failed for debugging purposes.
- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error {
    if (error) {
        [self setStatus:@"Error discovering characteristics"];
        return;
    }

    for (CBCharacteristic *characteristic in service.characteristics) {
        if ([characteristic.UUID isEqual:self.characteristicUUID]) {
            [self setStatus:@"Connected"];
            self.characteristic = characteristic;
        }
    }
}

// Function to update the status label when a value is sent to the peripheral.
- (void)peripheral:(CBPeripheral *)peripheral didWriteValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error {
    if (error) {
        [self setStatus:error.description];
    } else {
        [self setStatus:@"Sent"];
    }
}

// Function to write the broadcasted value to the BLE peripheral. Specifically keeps a timer for the last value sent to avoid overloading the Arduino NANO with commands. Will update the status label when a value is sending. Will log when a command is skipped.
- (void)writeString:(NSString *)string {
    NSDate *now = [NSDate date];

    // Check if lastCommandSendTime is set and the difference is less than 32 ms
    if (self.lastCommandSendTime && [now timeIntervalSinceDate:self.lastCommandSendTime] < MIN_COMMAND_SEND_PERIOD) {
        NSLog(@"Command skipped, too soon.");
        return;
    }

    self.lastCommandSendTime = now;

    if (self.peripheral && self.peripheral.state == CBPeripheralStateConnected) {
        NSData *dataToWrite = [string dataUsingEncoding:NSUTF8StringEncoding];
        [self.peripheral writeValue:dataToWrite
                  forCharacteristic:self.characteristic
                               type:CBCharacteristicWriteWithResponse];
    }
    [self setStatus:@"Sending"];
}

// *** Interactions ***

// Button for Sending a specific string set in the adjacent text field. This action broadcasts the string.
- (IBAction)sendClicked:(id)sender {
    [self writeString:self.inputTextField.text];
}

// **Steering Slider and Power Slider Logic**

// Stops the previous timer and starts a new one. Specifically for the steering slider.
- (void)startSteeringSliderTimer {
    [self.sliderTimerSteer invalidate];
    self.sliderTimerSteer = [NSTimer scheduledTimerWithTimeInterval:0.05 target:self selector:@selector(updateSteeringSliderValue) userInfo:nil repeats:YES];
}

// Function specifically for the steering slider to make the slider trend to zero. Every instance this function is called, a steering value change is broadcasted.
- (void)updateSteeringSliderValue {
    if (self.motorSteeringSlider.value > 1) {
        self.motorSteeringSlider.value -= 1;
        [self motorSteeringSliderValueChanged:nil];
    } else if (self.motorSteeringSlider.value < -1) {
        self.motorSteeringSlider.value += 1;
        [self motorSteeringSliderValueChanged:nil];
    } else {
        [self.sliderTimerSteer invalidate];
        self.sliderTimerSteer = nil;
    }
}

// Stops the previous timer and starts a new one. Specifically for the power slider.
- (void)startPowerSliderTimer {
    [self.sliderTimerPower invalidate];
    self.sliderTimerPower = [NSTimer scheduledTimerWithTimeInterval:0.05 target:self selector:@selector(updatePowerSliderValue) userInfo:nil repeats:YES];
}

// Function specifically for the power slider to make the slider trend to zero. Every instance this function is called, a power value change is broadcasted.
- (void)updatePowerSliderValue {
    if (self.motorPowerSlider.value > 1) {
        self.motorPowerSlider.value -= 1;
        [self motorPowerSliderValueChanged:nil];
    } else if (self.motorPowerSlider.value < -1) {
        self.motorPowerSlider.value += 1;
        [self motorPowerSliderValueChanged:nil];
    } else {
        [self.sliderTimerPower invalidate];
        self.sliderTimerPower = nil;
    }
}

// Function that stops voth steering and power timers.
- (void)stopSliderTimer:(NSTimer *)timer {
    [self.sliderTimerSteer invalidate];
    self.sliderTimerSteer = nil;
    [self.sliderTimerPower invalidate];
    self.sliderTimerPower = nil;
}


// Each action by each controller on the screen will send a specific string attached with an integer. The format is [L][Number] where [L] is a character identifier tag and [Number] is the attached data to the tag. Everything is string based which is interpreted in the Arduino NANO as commands. 
// P - Motor Power, S - Motor Steering, L - Left Lights, R - Right Lights, M - Manual Light Switch, A - Auto Light Switch.

- (IBAction)motorPowerSliderValueChanged:(id)sender {
    [self writeString:[NSString stringWithFormat:@"P%d", (int)self.motorPowerSlider.value]];
}

- (IBAction)motorSteeringSliderValueChanged:(id)sender {
    [self writeString:[NSString stringWithFormat:@"S%d", (int)self.motorSteeringSlider.value]];
}

- (IBAction)signalLeftChanged:(id)sender {
    if (self.toggleSignalLeft.isOn == TRUE){
        [self writeString:[NSString stringWithFormat:@"L%d", (int)1]];
    } else {
        [self writeString:[NSString stringWithFormat:@"L%d", (int)0]];
    }
}

- (IBAction)signalRightChanged:(id)sender {
    if (self.toggleSignalRight.isOn == TRUE){
        [self writeString:[NSString stringWithFormat:@"R%d", (int)1]];
    } else {
        [self writeString:[NSString stringWithFormat:@"R%d", (int)0]];
    }
}

- (IBAction)manualChanged:(id)sender {
    if (self.toggleManual.isOn == TRUE){
        [self writeString:[NSString stringWithFormat:@"M%d", (int)1]];
    } else {
        [self writeString:[NSString stringWithFormat:@"M%d", (int)0]];
    }
}

- (IBAction)autoChanged:(id)sender {
    if (self.toggleAuto.isOn == TRUE){
        [self writeString:[NSString stringWithFormat:@"A%d", (int)1]];
    } else {
        [self writeString:[NSString stringWithFormat:@"A%d", (int)0]];
    }
}

@end
