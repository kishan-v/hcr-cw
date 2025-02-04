SERVER_CONFIG = {
    "ip": "130.162.176.219",
    "port": 5000,
}

# MACBOOK WEBCAM
# DEVICE_CONFIG = {
#     "latency": 50,  # 50ms
#     "input_opts": [
#         "-f",
#         "avfoundation",
#         # "-framerate",
#         # "30",
#         # "-i",
#         # "1",
#         "-framerate",
#         "29.97",
#         "-i",
#         "RICOH THETA Z1:none",
#         # "-vf",
#         # (
#         #     "scale=1280:720,"
#         #     "drawtext=fontfile=/Library/Fonts/Arial.ttf: "
#         #     "text='%{pts\\:hms}': "
#         #     "x=10: y=30: fontsize=24: fontcolor=white: box=1: boxcolor=black@0.5"
#         # ),
#     ],
#     "output_opts": [
#         "-f",
#         "mpegts",
#         "-c:v",
#         "h264",
#         "-preset",
#         "ultrafast",
#         "-tune",
#         "zerolatency",
#     ],
# }

DEVICE_CONFIG = {
    "latency": 50,  # 50ms latency
    "input_opts": [
        "-f", "avfoundation",
        "-framerate", "29.97",
        "-pixel_format", "uyvy422",  # Added to force the correct pixel format on input
        "-i", "RICOH THETA Z1:none",
    ],
    "output_opts": [
        "-vf", "format=yuv422p",      # Added to ensure proper conversion before encoding
        "-f", "mpegts",
        "-c:v", "h264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
    ],
}

# RICOH THETA 360
# DEVICE_CONFIG = {
#     "latency": 50,  # 50ms
#     "input_opts": [
#         "-f",
#         "v4l2",
#         "-input_format",
#         "mjpeg",
#         "-video_size",
#         "3840x1920",  # TODO: confirm
#         "-framerate",
#         "30",
#         "-i",
#         "/dev/video0",
#     ],
#     "output_opts": [
#         "-c:v",
#         "libx264",
#         "-preset",
#         "ultrafast",
#         "-tune",
#         "zerolatency",
#         "-crf",
#         "18",
#         "-f",
#         "mpegts",
#     ],
# }
