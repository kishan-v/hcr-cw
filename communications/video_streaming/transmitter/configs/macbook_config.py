MACBOOK_CONFIG = {
    "latency": 50,
    "input_opts": [
        "-f",
        "avfoundation",
        "-framerate",
        "30",
        "-i",
        "0",
        "-vf",
        (
            "scale=1280:720,"
            "drawtext=fontfile=/Library/Fonts/Arial.ttf: "
            "text='%{pts\\:hms}': "
            "x=10: y=30: fontsize=24: fontcolor=white: box=1: boxcolor=black@0.5"
        ),
    ],
    "output_opts": [
        "-f",
        "mpegts",
        "-c:v",
        "h264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
    ],
}
