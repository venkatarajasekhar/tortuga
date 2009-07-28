//
//  OscilloscopeController.h
//  BlackfinScope
//
//  Created by Leo Singer on 7/27/09.
//  Copyright 2009 Robotics@Maryland. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import "scope.h"
#import "OscilloscopeView.h"


@interface OscilloscopeController : ramsonarscopeViewer <ramsonarscopeViewer> {

    id<ICECommunicator> communicator;
    id<ICEObjectAdapter> adapter;
    id<ramsonarscopeOscilloscopePrx> oscPrx;
    
    IBOutlet NSSegmentedControl* triggerModeControl;
    IBOutlet NSProgressIndicator* triggerProgressIndicator;
    IBOutlet OscilloscopeView* view;
    
}
- (void) NotifyCapture:(ICECurrent *)current;
- (IBAction)scopeModeChanged: (id)sender;
- (IBAction)triggerLevelChanged: (id)sender;
- (IBAction)triggerChannelChanged: (id)sender;
- (IBAction)triggerModeChanged: (id)sender;
- (IBAction)triggerSlopeChanged: (id)sender;
- (IBAction)horizontalZoomChanged: (id)sender;

- (void) updateSpectrum;
@end
