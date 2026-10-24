import sensor
import time
from machine import UART


# ============================================================
# OPENMV RT1062 - 4x3 MOSAIC SCANNER + UART
#
# COLORS:
#   1 = Yellow
#   2 = Blue
#   3 = Green
#   4 = White
#
# Camera is mounted upside down.
#
# UART:
#
# ESP32 -> OpenMV:
#   S,1\n
#
# OpenMV -> ESP32:
#   M,1,2,4,3,3,4,3,1,4,1,4,2,1\n
#
# Matrix is sent ROW-MAJOR and already rotated 180 degrees
# into ROBOT orientation.
# ============================================================


# ============================================================
# QUICK SETTINGS
# ============================================================

DEBUG = True


# ============================================================
# COLOR THRESHOLDS
# ============================================================

# Yellow
YELLOW_B_MIN = 13

# White
WHITE_L_MIN = 45

# Blue
BLUE_L_MAX = 40
BLUE_B_MAX = -10

# Green
GREEN_A_MAX = -10


# ============================================================
# VOTING
# ============================================================

VOTE_FRAMES = 3

# Minimum agreement out of 3
MIN_VOTE_CONFIDENCE = 2

# How many complete scan attempts are allowed for each S,id
MAX_REQUEST_SCAN_ATTEMPTS = 2


# ============================================================
# BLACK BOARD DETECTION
# ============================================================

BLACK_GRAY_MAX = 60

SEARCH_ROI = (
    35,
    55,
    250,
    115
)

PIXELS_THRESHOLD = 80
AREA_THRESHOLD = 150


# ------------------------------------------------------------
# Valid mosaic bounding box
# ------------------------------------------------------------

MIN_BOARD_W = 100
MAX_BOARD_W = 260

MIN_BOARD_H = 40
MAX_BOARD_H = 130

MIN_BOARD_ASPECT = 1.5
MAX_BOARD_ASPECT = 4.5


# ============================================================
# CALIBRATED TILE POSITIONS
# ============================================================

ROW_Y = [
    0.31,
    0.59,
    0.85
]


ROW_X = [
    [0.16, 0.39, 0.61, 0.84],
    [0.18, 0.40, 0.60, 0.82],
    [0.20, 0.41, 0.59, 0.80]
]


# ============================================================
# LOCAL TILE SEARCH
# ============================================================

PATCH_W = 6
PATCH_H = 6

X_OFFSETS = (
    -6,
    0,
    6
)

Y_OFFSETS = (
    -5,
    0,
    5
)


# ============================================================
# UART
# ============================================================

uart = UART(
    1,
    baudrate=115200
)

uart_buffer = b""


# ============================================================
# CAMERA
# ============================================================

sensor.reset()

sensor.set_pixformat(
    sensor.RGB565
)

sensor.set_framesize(
    sensor.QVGA
)


# Let camera auto-adjust
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)

sensor.skip_frames(
    time=2000
)


# Save selected values
gain = sensor.get_gain_db()
exposure = sensor.get_exposure_us()
rgb_gain = sensor.get_rgb_gain_db()


# Freeze camera
sensor.set_auto_gain(
    False,
    gain_db=gain
)

sensor.set_auto_exposure(
    False,
    exposure_us=exposure
)

sensor.set_auto_whitebal(
    False,
    rgb_gain_db=rgb_gain
)

time.sleep_ms(300)


if DEBUG:

    print("")
    print("==============================")
    print("OPENMV SCANNER READY")
    print("==============================")

    print("Gain:", gain)
    print("Exposure:", exposure)
    print("RGB:", rgb_gain)


# ============================================================
# BLOB HELPERS
# ============================================================

def blob_box(b):

    return (
        b[0],
        b[1],
        b[2],
        b[3]
    )


# ============================================================
# VALID BOARD
# ============================================================

def valid_board(b):

    w = b[2]
    h = b[3]


    if w < MIN_BOARD_W:
        return False

    if w > MAX_BOARD_W:
        return False


    if h < MIN_BOARD_H:
        return False

    if h > MAX_BOARD_H:
        return False


    if h == 0:
        return False


    aspect = w / h


    if aspect < MIN_BOARD_ASPECT:
        return False

    if aspect > MAX_BOARD_ASPECT:
        return False


    return True


# ============================================================
# FIND BLACK BOARD
# ============================================================

def find_board(img):

    gray = img.to_grayscale(
        copy=True
    )


    blobs = gray.find_blobs(

        [
            (
                0,
                BLACK_GRAY_MAX
            )
        ],

        roi=SEARCH_ROI,

        pixels_threshold=PIXELS_THRESHOLD,

        area_threshold=AREA_THRESHOLD,

        merge=False
    )


    best = None
    best_area = 0


    for b in blobs:


        if not valid_board(b):
            continue


        area = (
            b[2]
            * b[3]
        )


        if area > best_area:

            best_area = area
            best = b


    return best


# ============================================================
# CALCULATE TILE CENTERS
# ============================================================

def get_centers(board):

    x = board[0]
    y = board[1]

    w = board[2]
    h = board[3]


    centers = [
        [None, None, None, None],
        [None, None, None, None],
        [None, None, None, None]
    ]


    for row in range(3):

        cy = int(
            y
            + ROW_Y[row] * h
        )


        for col in range(4):

            cx = int(
                x
                + ROW_X[row][col] * w
            )


            centers[row][col] = (
                cx,
                cy
            )


    return centers


# ============================================================
# COLOR CLASSIFICATION
# ============================================================

def classify_color(L, A, B):


    # Yellow
    if B > YELLOW_B_MIN:

        return 1


    # Green
    if A < GREEN_A_MAX:

        return 3


    # White
    if L > WHITE_L_MIN:

        return 4


    # Blue
    if (
        L < BLUE_L_MAX
        and B < BLUE_B_MAX
    ):

        return 2


    # Fallback
    return 4


# ============================================================
# SAMPLE ONE TILE
# ============================================================

def sample_tile(
    img,
    cx,
    cy
):

    image_w = img.width()
    image_h = img.height()


    best_score = -999999

    best_L = 0
    best_A = 0
    best_B = 0

    best_roi = None


    # --------------------------------------------------------
    # Search 3x3 area around predicted center
    # --------------------------------------------------------

    for dy in Y_OFFSETS:

        for dx in X_OFFSETS:


            px = (
                cx
                + dx
                - PATCH_W // 2
            )


            py = (
                cy
                + dy
                - PATCH_H // 2
            )


            if px < 0:
                continue

            if py < 0:
                continue

            if px + PATCH_W >= image_w:
                continue

            if py + PATCH_H >= image_h:
                continue


            roi = (
                px,
                py,
                PATCH_W,
                PATCH_H
            )


            stats = img.get_statistics(
                roi=roi
            )


            L = stats.l_mean
            A = stats.a_mean
            B = stats.b_mean


            # ------------------------------------------------
            # Prefer real tile surface over black plastic
            # ------------------------------------------------

            chroma = (
                abs(A)
                + abs(B)
            )


            score = (
                L
                + 3 * chroma
            )


            if score > best_score:

                best_score = score

                best_L = L
                best_A = A
                best_B = B

                best_roi = roi


    color = classify_color(
        best_L,
        best_A,
        best_B
    )


    return (
        color,
        (
            best_L,
            best_A,
            best_B
        ),
        best_roi
    )


# ============================================================
# READ ONE CAMERA FRAME
# ============================================================

def read_frame(
    img,
    board
):

    centers = get_centers(
        board
    )


    matrix = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ]


    labs = [
        [None, None, None, None],
        [None, None, None, None],
        [None, None, None, None]
    ]


    rois = [
        [None, None, None, None],
        [None, None, None, None],
        [None, None, None, None]
    ]


    for row in range(3):

        for col in range(4):


            cx, cy = (
                centers[row][col]
            )


            (
                color,
                lab,
                roi
            ) = sample_tile(
                img,
                cx,
                cy
            )


            matrix[row][col] = color
            labs[row][col] = lab
            rois[row][col] = roi


    return (
        matrix,
        labs,
        rois,
        centers
    )


# ============================================================
# ROTATE 180 DEGREES
#
# Camera is physically upside down.
# ============================================================

def rotate_180(matrix):

    return [
        matrix[2][::-1],
        matrix[1][::-1],
        matrix[0][::-1]
    ]


# ============================================================
# SCAN MOSAIC
# ============================================================

def scan_mosaic():

    # --------------------------------------------------------
    # Vote storage
    # --------------------------------------------------------

    votes = []


    for row in range(3):

        row_votes = []


        for col in range(4):

            # Index 0 unused
            row_votes.append(
                [0, 0, 0, 0, 0]
            )


        votes.append(
            row_votes
        )


    # --------------------------------------------------------
    # LAB accumulation for debug only
    # --------------------------------------------------------

    if DEBUG:

        lab_sum = []


        for row in range(3):

            lab_row = []


            for col in range(4):

                lab_row.append(
                    [0, 0, 0]
                )


            lab_sum.append(
                lab_row
            )


    else:

        lab_sum = None


    valid_frames = 0
    attempts = 0

    max_attempts = (
        VOTE_FRAMES * 4
    )


    last_img = None
    last_board = None
    last_rois = None
    last_centers = None
    last_camera_matrix = None


    # ========================================================
    # OBTAIN 3 VALID BOARD FRAMES
    # ========================================================

    while (
        valid_frames < VOTE_FRAMES
        and attempts < max_attempts
    ):


        attempts += 1


        img = sensor.snapshot()


        board = find_board(
            img
        )


        if board is None:

            continue


        (
            matrix,
            labs,
            rois,
            centers
        ) = read_frame(
            img,
            board
        )


        # ----------------------------------------------------
        # Add votes
        # ----------------------------------------------------

        for row in range(3):

            for col in range(4):


                color = matrix[row][col]


                votes[row][col][color] += 1


                if DEBUG:

                    L, A, B = labs[row][col]


                    lab_sum[row][col][0] += L
                    lab_sum[row][col][1] += A
                    lab_sum[row][col][2] += B


        valid_frames += 1


        last_img = img
        last_board = board
        last_rois = rois
        last_centers = centers
        last_camera_matrix = matrix


    # --------------------------------------------------------
    # Not enough valid board frames
    # --------------------------------------------------------

    if valid_frames < VOTE_FRAMES:

        return None


    # ========================================================
    # BUILD FINAL CAMERA MATRIX
    # ========================================================

    camera_matrix = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ]


    camera_confidence = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ]


    if DEBUG:

        camera_labs = [
            [None, None, None, None],
            [None, None, None, None],
            [None, None, None, None]
        ]


    else:

        camera_labs = None


    for row in range(3):

        for col in range(4):


            best_color = 1
            best_votes = votes[row][col][1]


            for color in range(
                2,
                5
            ):


                if (
                    votes[row][col][color]
                    > best_votes
                ):

                    best_color = color

                    best_votes = (
                        votes[row][col][color]
                    )


            camera_matrix[row][col] = (
                best_color
            )


            camera_confidence[row][col] = (
                best_votes
            )


            if DEBUG:

                camera_labs[row][col] = (

                    lab_sum[row][col][0]
                    // valid_frames,

                    lab_sum[row][col][1]
                    // valid_frames,

                    lab_sum[row][col][2]
                    // valid_frames
                )


    # ========================================================
    # ROTATE TO ROBOT ORIENTATION
    # ========================================================

    robot_matrix = rotate_180(
        camera_matrix
    )


    robot_confidence = rotate_180(
        camera_confidence
    )


    if DEBUG:

        robot_labs = rotate_180(
            camera_labs
        )


    else:

        robot_labs = None


    return (
        robot_matrix,
        robot_confidence,
        robot_labs,
        last_img,
        last_board,
        last_rois,
        last_centers,
        last_camera_matrix
    )


# ============================================================
# VALIDATE SCAN
# ============================================================

def scan_is_valid(
    matrix,
    confidence
):


    for row in range(3):

        for col in range(4):


            color = matrix[row][col]


            if (
                color < 1
                or color > 4
            ):

                return False


            if (
                confidence[row][col]
                < MIN_VOTE_CONFIDENCE
            ):

                return False


    return True


# ============================================================
# DEBUG DRAWING
#
# Drawing stays in camera orientation.
# ============================================================

def draw_debug(
    img,
    board,
    camera_matrix,
    rois,
    centers
):


    if not DEBUG:
        return


    # Search region
    img.draw_rectangle(
        SEARCH_ROI,
        color=(255, 0, 0),
        thickness=1
    )


    # Board bounding box
    img.draw_rectangle(
        blob_box(board),
        color=(255, 255, 0),
        thickness=2
    )


    # --------------------------------------------------------
    # Tile overlays
    # --------------------------------------------------------

    for row in range(3):

        for col in range(4):


            color = (
                camera_matrix[row][col]
            )


            if color == 1:

                draw_color = (
                    255,
                    255,
                    0
                )


            elif color == 2:

                draw_color = (
                    0,
                    0,
                    255
                )


            elif color == 3:

                draw_color = (
                    0,
                    255,
                    0
                )


            else:

                draw_color = (
                    255,
                    255,
                    255
                )


            # Selected LAB sample
            img.draw_rectangle(
                rois[row][col],
                color=draw_color,
                thickness=2
            )


            # Predicted tile center
            img.draw_cross(
                centers[row][col],
                color=(0, 255, 0)
            )


# ============================================================
# GET ONE VALID SCAN
# ============================================================

def get_valid_scan():


    for attempt in range(
        MAX_REQUEST_SCAN_ATTEMPTS
    ):


        if DEBUG:

            print("")
            print(
                "SCAN ATTEMPT:",
                attempt + 1
            )


        result = scan_mosaic()


        if result is None:

            if DEBUG:

                print(
                    "BOARD DETECTION FAILED"
                )

            continue


        (
            matrix,
            confidence,
            labs,
            img,
            board,
            rois,
            centers,
            camera_matrix
        ) = result


        valid = scan_is_valid(
            matrix,
            confidence
        )


        # ----------------------------------------------------
        # Debug framebuffer
        # ----------------------------------------------------

        if DEBUG:

            draw_debug(
                img,
                board,
                camera_matrix,
                rois,
                centers
            )


            print("")
            print("==============================")
            print("MOSAIC - ROBOT ORIENTATION")
            print("==============================")


            print(matrix[0])
            print(matrix[1])
            print(matrix[2])


            print("")
            print("CONFIDENCE")


            print(confidence[0])
            print(confidence[1])
            print(confidence[2])


            print("")
            print("LAB VALUES")


            print(labs[0])
            print(labs[1])
            print(labs[2])


            if valid:

                print("")
                print("SCAN VALID")


            else:

                print("")
                print(
                    "SCAN LOW CONFIDENCE"
                )


        # ----------------------------------------------------
        # Only return valid result.
        # ----------------------------------------------------

        if valid:

            return matrix


    return None


# ============================================================
# FLATTEN MATRIX
#
# [
#   [2,4,3,3],
#   [4,3,1,4],
#   [1,4,2,1]
# ]
#
# ->
#
# 2,4,3,3,4,3,1,4,1,4,2,1
# ============================================================

def flatten_matrix(matrix):

    result = []


    for row in range(3):

        for col in range(4):

            result.append(
                matrix[row][col]
            )


    return result


# ============================================================
# SEND MATRIX
# ============================================================

def send_matrix(
    request_id,
    matrix
):


    flat = flatten_matrix(
        matrix
    )


    message = (
        "M,"
        + str(request_id)
    )


    for value in flat:

        message += (
            ","
            + str(value)
        )


    message += "\n"


    uart.write(
        message.encode()
    )


    if DEBUG:

        print("")
        print(
            "TX:",
            message.strip()
        )


# ============================================================
# UART MAIN LOOP
# ============================================================

while True:


    # --------------------------------------------------------
    # RECEIVE UART DATA
    # --------------------------------------------------------

    if uart.any():


        data = uart.read()


        if data:

            uart_buffer += data


    # --------------------------------------------------------
    # PROCESS COMPLETE COMMANDS
    # --------------------------------------------------------

    while b"\n" in uart_buffer:


        line, uart_buffer = uart_buffer.split(
            b"\n",
            1
        )


        line = line.strip()


        if DEBUG:

            print("")
            print(
                "RX:",
                line
            )


        try:


            text = line.decode()

            parts = text.split(",")


            # =================================================
            # SCAN COMMAND
            #
            # S,<request_id>
            # =================================================

            if (
                len(parts) == 2
                and parts[0] == "S"
            ):


                request_id = int(
                    parts[1]
                )


                # =============================================
                # PERFORM REAL CAMERA SCAN
                # =============================================

                matrix = get_valid_scan()


                # =============================================
                # SEND ONLY VALID SCANS
                # =============================================

                if matrix is not None:


                    send_matrix(
                        request_id,
                        matrix
                    )


                else:


                    if DEBUG:

                        print("")
                        print(
                            "NO VALID SCAN - "
                            "NO MATRIX SENT"
                        )


        except Exception as e:


            if DEBUG:

                print(
                    "UART ERROR:",
                    e
                )


    # Keep UART loop responsive
    time.sleep_ms(2)
