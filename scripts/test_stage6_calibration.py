#!/usr/bin/env python3
"""Complete calibration identity without treating acquisition time as version."""
import copy
import unittest
from sensor_msgs.msg import CameraInfo
from pick_place_perception.geometry_monitor import calibration_signature


class CalibrationIdentityTests(unittest.TestCase):
    def setUp(self):
        self.info=CameraInfo(width=640,height=480,distortion_model='plumb_bob')
        self.info.header.frame_id='camera_optical'
        self.info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.]
        self.info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        self.info.p=[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.]

    def test_acquisition_stamp_not_version(self):
        original=calibration_signature(self.info)
        self.info.header.stamp.sec=100
        self.assertEqual(original,calibration_signature(self.info))

    def test_every_calibration_component_bound(self):
        for field in ('k','r','p','d','width','height','distortion_model','binning_x','binning_y'):
            with self.subTest(field=field):
                changed=copy.deepcopy(self.info)
                value=getattr(changed,field)
                if field=='d':changed.d=[.001]
                elif field in ('k','r','p'):value[0]+=.001
                elif isinstance(value,str):setattr(changed,field,value+'changed')
                else:setattr(changed,field,value+1)
                self.assertNotEqual(calibration_signature(self.info),calibration_signature(changed))
        for field in ('x_offset','y_offset','width','height','do_rectify'):
            changed=copy.deepcopy(self.info)
            value=getattr(changed.roi,field)
            setattr(changed.roi,field,not value if isinstance(value,bool) else value+1)
            self.assertNotEqual(calibration_signature(self.info),calibration_signature(changed))


if __name__=='__main__':unittest.main(verbosity=2)
