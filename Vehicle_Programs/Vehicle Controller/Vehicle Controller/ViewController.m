//
//  ViewController.m
//  Vehicle Controller
//
//  Created by Nathan Fikes on 11/20/23.
//

@import CoreBluetooth;
#import "ViewController.h"

#define MIN_COMMAND_SEND_PERIOD 0.032

@interface ViewController ()

@property (weak, nonatomic) IBOutlet UILabel *statusLabel;
@property (weak, nonatomic) IBOutlet UITextField *inputTextField;


@property (strong, nonatomic) CBCentralManager *centralManager;
@property (strong, nonatomic) CBUUID *serviceUUID;
@property (strong, nonatomic) CBUUID *characteristicUUID;
@property (strong, nonatomic) CBPeripheral *peripheral;
@property (strong, nonatomic) CBCharacteristic *characteristic;
@property (weak, nonatomic) IBOutlet UISlider *motorPowerSlider;
@property (weak, nonatomic) IBOutlet UISlider *motorSteeringSlider;

@property (strong, nonatomic) NSTimer *sliderTimerSteer;
@property (strong, nonatomic) NSTimer *sliderTimerPower;

@property (strong, nonatomic) NSDate *lastCommandSendTime;

@property (weak, nonatomic) IBOutlet UISwitch *toggleSignalLeft;
@property (weak, nonatomic) IBOutlet UISwitch *toggleSignalRight;
@property (weak, nonatomic) IBOutlet UISwitch *toggleManual;
@property (weak, nonatomic) IBOutlet UISwitch *toggleAuto;


@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    [self setupKeyboardDismissRecognizer];
    
    [self.motorSteeringSlider addTarget:self action:@selector(startSteeringSliderTimer) forControlEvents:UIControlEventTouchUpInside];
    [self.motorSteeringSlider addTarget:self action:@selector(startSteeringSliderTimer) forControlEvents:UIControlEventTouchUpOutside];
    [self.motorSteeringSlider addTarget:self action:@selector(stopSliderTimer:) forControlEvents:UIControlEventTouchDown];

    [self.motorPowerSlider addTarget:self action:@selector(startPowerSliderTimer) forControlEvents:UIControlEventTouchUpInside];
    [self.motorPowerSlider addTarget:self action:@selector(startPowerSliderTimer) forControlEvents:UIControlEventTouchUpOutside];
    [self.motorPowerSlider addTarget:self action:@selector(stopSliderTimer:) forControlEvents:UIControlEventTouchDown];
    
    self.motorPowerSlider.transform = CGAffineTransformMakeRotation(-M_PI_2);
    self.motorSteeringSlider.transform = CGAffineTransformMakeRotation(-M_PI_2);
    
    self.serviceUUID = [CBUUID UUIDWithString:@"4fafc201-1fb5-459e-8fcc-c5c9c331915c"];
    self.characteristicUUID = [CBUUID UUIDWithString:@"beb5483e-36e1-4688-b7f5-ea07361b26a8"];
    [self connect];
}

- (void)setupKeyboardDismissRecognizer {
    UITapGestureRecognizer *tapRecognizer = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(dismissKeyboard)];
    [self.view addGestureRecognizer:tapRecognizer];
}

- (void)dismissKeyboard {
    [self.view endEditing:YES];
}

- (void)setStatus:(NSString*)status {
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
        self.statusLabel.text = status;
    });
}

- (void)connect {
    [self setStatus:@"Connecting..."];
    if (!self.centralManager) {
        self.centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:nil options:nil];
    } else if (self.centralManager.state == CBManagerStatePoweredOn) {
        [self startScanning];
    }
}

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    if (central.state == CBManagerStatePoweredOn) {
        [self startScanning];
    }
}

- (void)startScanning {
    [self setStatus:@"Scanning..."];
    [self.centralManager scanForPeripheralsWithServices:@[self.serviceUUID] options:nil];
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary<NSString *,id> *)advertisementData RSSI:(NSNumber *)RSSI {
    if (!self.peripheral || self.peripheral.state == CBPeripheralStateDisconnected) {
        [self.centralManager stopScan];
        self.peripheral = peripheral;
        [self.centralManager connectPeripheral:peripheral options:nil];
    }
}

- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral {
    self.peripheral = peripheral;
    peripheral.delegate = self;
    [peripheral discoverServices:@[self.serviceUUID]];
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    [self setStatus:@"Disconnected"];
    self.peripheral = nil;
    [self connect];
}


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

- (void)peripheral:(CBPeripheral *)peripheral didWriteValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error {
    if (error) {
        [self setStatus:error.description];
    } else {
        [self setStatus:@"Sent"];
    }
}

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

//Interactions

- (IBAction)sendClicked:(id)sender {
    [self writeString:self.inputTextField.text];
}

//Steering Slider and Power Slider Logic

- (void)startSteeringSliderTimer {
    [self.sliderTimerSteer invalidate];
    self.sliderTimerSteer = [NSTimer scheduledTimerWithTimeInterval:0.05 target:self selector:@selector(updateSteeringSliderValue) userInfo:nil repeats:YES];
}

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

- (void)startPowerSliderTimer {
    [self.sliderTimerPower invalidate];
    self.sliderTimerPower = [NSTimer scheduledTimerWithTimeInterval:0.05 target:self selector:@selector(updatePowerSliderValue) userInfo:nil repeats:YES];
}

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


- (void)stopSliderTimer:(NSTimer *)timer {
    [self.sliderTimerSteer invalidate];
    self.sliderTimerSteer = nil;
    [self.sliderTimerPower invalidate];
    self.sliderTimerPower = nil;
}

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
