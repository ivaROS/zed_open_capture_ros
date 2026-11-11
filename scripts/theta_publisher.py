#!/usr/bin/env python3
"""Publish the RICOH THETA live stream (equirectangular) as JPEG images.

The camera must be in live streaming mode (USB product id 0x2717 for the
THETA X). The H.264 stream is read through libuvc (libuvc-theta build),
decoded with GStreamer, reduced to the requested rate, and published as
sensor_msgs/CompressedImage on <topic>/compressed.

Parameters:
  mode          4k (3840x1920) or 2k (1920x960), default 4k
  rate          output rate in Hz, default 1.0 (0.1 also works)
  jpeg_quality  default 90
  frame_id      default theta_link
  topic         default /theta/image_raw
  decoder       GStreamer H.264 decoder, default: vah264dec if present,
                otherwise avdec_h264

Header stamps are the host time when each H.264 frame arrived over USB. The
camera's stitching and encoding delay is not removed.
"""

import ctypes
import ctypes.util
import fractions
import threading
import time

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

RICOH_VID = 0x05ca
THETA_PIDS = (0x2717, 0x2715, 0x2712)  # X, Z1, V in live streaming mode
UVC_FRAME_FORMAT_H264 = 8
MODES = {'4k': (3840, 1920, 29), '2k': (1920, 960, 29)}


class UvcFrame(ctypes.Structure):
    # Leading fields of struct uvc_frame; only data and data_bytes are used.
    _fields_ = [('data', ctypes.c_void_p),
                ('data_bytes', ctypes.c_size_t),
                ('width', ctypes.c_uint32),
                ('height', ctypes.c_uint32)]


FRAME_CB = ctypes.CFUNCTYPE(None, ctypes.POINTER(UvcFrame), ctypes.c_void_p)


def load_libuvc():
    for name in ('/usr/local/lib/libuvc.so', ctypes.util.find_library('uvc')):
        if name:
            try:
                lib = ctypes.CDLL(name)
                break
            except OSError:
                continue
    else:
        raise RuntimeError('libuvc not found')
    vp = ctypes.c_void_p
    lib.uvc_init.argtypes = [ctypes.POINTER(vp), vp]
    lib.uvc_exit.argtypes = [vp]
    lib.uvc_find_device.argtypes = [vp, ctypes.POINTER(vp), ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
    lib.uvc_open.argtypes = [vp, ctypes.POINTER(vp)]
    lib.uvc_close.argtypes = [vp]
    lib.uvc_unref_device.argtypes = [vp]
    lib.uvc_get_stream_ctrl_format_size.argtypes = [vp, vp, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.uvc_start_streaming.argtypes = [vp, vp, FRAME_CB, vp, ctypes.c_uint8]
    lib.uvc_stop_streaming.argtypes = [vp]
    lib.uvc_strerror.argtypes = [ctypes.c_int]
    lib.uvc_strerror.restype = ctypes.c_char_p
    return lib


class ThetaPublisher(Node):

    def __init__(self):
        super().__init__('theta')
        mode = self.declare_parameter('mode', '4k').value
        rate = float(self.declare_parameter('rate', 1.0).value)
        quality = int(self.declare_parameter('jpeg_quality', 90).value)
        self.frame_id = self.declare_parameter('frame_id', 'theta_link').value
        topic = self.declare_parameter('topic', '/theta/image_raw').value
        decoder = self.declare_parameter('decoder', '').value
        if mode not in MODES:
            raise RuntimeError('mode must be 4k or 2k')
        width, height, fps = MODES[mode]

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(CompressedImage, topic + '/compressed', qos)

        Gst.init(None)
        if not decoder:
            decoder = 'vah264dec' if Gst.ElementFactory.find('vah264dec') else 'avdec_h264'
        out_rate = fractions.Fraction(rate).limit_denominator(1000)
        pipeline = (
            'appsrc name=src is-live=true format=time '
            'caps=video/x-h264,stream-format=byte-stream,framerate=30000/1001 '
            '! h264parse ! %s ! videorate drop-only=true '
            '! video/x-raw,framerate=%d/%d ! videoconvert ! jpegenc quality=%d '
            '! appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true'
            % (decoder, out_rate.numerator, out_rate.denominator, quality))
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsrc = self.pipeline.get_by_name('src')
        sink = self.pipeline.get_by_name('sink')
        sink.connect('new-sample', self.on_sample)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        self.pipeline.set_state(Gst.State.PLAYING)

        # Buffer PTS = host arrival time relative to t0, so stamps survive decoding.
        self.t0_ns = time.time_ns()
        self.frames_in = 0
        self.frames_out = 0
        self.lock = threading.Lock()

        self.uvc = load_libuvc()
        self.ctx = ctypes.c_void_p()
        self.dev = ctypes.c_void_p()
        self.devh = ctypes.c_void_p()
        self.ctrl = ctypes.create_string_buffer(64)  # uvc_stream_ctrl_t (48 bytes)
        self._check(self.uvc.uvc_init(ctypes.byref(self.ctx), None), 'uvc_init')
        for pid in THETA_PIDS:
            if self.uvc.uvc_find_device(self.ctx, ctypes.byref(self.dev), RICOH_VID, pid, None) == 0:
                break
        else:
            raise RuntimeError('No THETA in live streaming mode found')
        self._check(self.uvc.uvc_open(self.dev, ctypes.byref(self.devh)), 'uvc_open')
        self._check(self.uvc.uvc_get_stream_ctrl_format_size(
            self.devh, self.ctrl, UVC_FRAME_FORMAT_H264, width, height, fps), 'stream format')
        self._cb = FRAME_CB(self.on_uvc_frame)  # keep a reference
        self._check(self.uvc.uvc_start_streaming(self.devh, self.ctrl, self._cb, None, 0), 'start streaming')

        self.get_logger().info(
            'THETA %dx%d stream, decoder %s, publishing %s/compressed at %s Hz'
            % (width, height, decoder, topic, rate))
        self.create_timer(10.0, self.report)
        self.timer_bus = self.create_timer(0.5, lambda: self.poll_bus(bus))

    def _check(self, res, what):
        if res != 0:
            raise RuntimeError('%s failed: %s' % (what, self.uvc.uvc_strerror(res).decode()))

    def on_uvc_frame(self, frame_p, _user):
        frame = frame_p.contents
        if not frame.data or frame.data_bytes == 0:
            return
        data = ctypes.string_at(frame.data, frame.data_bytes)
        buf = Gst.Buffer.new_wrapped(data)
        buf.pts = time.time_ns() - self.t0_ns
        self.appsrc.emit('push-buffer', buf)
        with self.lock:
            self.frames_in += 1

    def on_sample(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        msg = CompressedImage()
        stamp_ns = self.t0_ns + buf.pts
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = bytes(info.data)
        buf.unmap(info)
        self.pub.publish(msg)
        with self.lock:
            self.frames_out += 1
        return Gst.FlowReturn.OK

    def poll_bus(self, bus):
        while True:
            msg = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.WARNING)
            if msg is None:
                return
            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self.get_logger().error('GStreamer: %s (%s)' % (err.message, dbg))
            else:
                err, dbg = msg.parse_warning()
                self.get_logger().warn('GStreamer: %s' % err.message)

    def report(self):
        with self.lock:
            fin, fout = self.frames_in, self.frames_out
            self.frames_in = self.frames_out = 0
        self.get_logger().info('last 10 s: %d stream frames in, %d images published' % (fin, fout))

    def close(self):
        if self.devh:
            self.uvc.uvc_stop_streaming(self.devh)
            self.uvc.uvc_close(self.devh)
        if self.dev:
            self.uvc.uvc_unref_device(self.dev)
        if self.ctx:
            self.uvc.uvc_exit(self.ctx)
        self.appsrc.emit('end-of-stream')
        self.pipeline.set_state(Gst.State.NULL)


def main():
    rclpy.init()
    node = ThetaPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
