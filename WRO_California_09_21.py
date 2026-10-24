from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
from pupremote_hub import PUPRemoteHub
from umath import pi
from sorter import *

hub = PrimeHub()
claw = Motor(Port.B, Direction.CLOCKWISE)
slide = Motor(Port.F, Direction.COUNTERCLOCKWISE)
stopper = Motor(Port.E, Direction.CLOCKWISE)

lmotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
rmotor = Motor(Port.C, Direction.CLOCKWISE)

db = DriveBase(lmotor, rmotor, 62.4, 158)
scan_id = 0
pr = PUPRemoteHub(
    Port.A,
    max_packet_size=16
)


pr.add_channel("lines", "bbB")

pr.add_command(
    "calib",
    from_hub_fmt="b"
)

pr.add_command(
    "scan",
    from_hub_fmt="B"
)

pr.add_channel(
    "cam",
    "13B"
)


WHEEL_DIAMETER_MM = 62.4


T_CONFIRM = 2

LOOP_DELAY = 3

def distance_travelled(left_start, right_start):
    left_degrees = abs(lmotor.angle() - left_start)
    right_degrees = abs(rmotor.angle() - right_start)

    average_degrees = (left_degrees + right_degrees) / 2

    return (
        average_degrees
        / 360
        * pi
        * WHEEL_DIAMETER_MM
    )


async def line(
    speed,
    distance,
    acceleration_distance=0,
    acceleration_speed=35,
    deceleration_distance=0,
    deceleration_speed=35,
    bias=0,
    skip=False,
    skip_distance=40,
    skip_error_jump=17, kp=-0.8, kd=-0.04,D_BRAKE = 0.04):
    """
    Line follow.

    speed:
        Normal line-following speed.

    distance:
        Distance in mm, or:
            "t" = stop at T junction
            "<" = stop at left branch
            ">" = stop at right branch

    acceleration_distance:
        Travel this many mm from the start at acceleration_speed.

    acceleration_speed:
        Speed during the initial acceleration section.

    deceleration_distance:
        AFTER travelling this many mm from the start,
        switch to deceleration_speed.

    deceleration_speed:
        Speed used after deceleration_distance is reached.

    bias:
        Positive = bias right.
        Negative = bias left.

    skip:
        If True, a sudden jump in line-position error causes
        the robot to drive straight for skip_distance mm.

    skip_distance:
        Distance in mm to push straight after detecting
        a sudden error jump.

    skip_error_jump:
        Minimum change in position error between consecutive
        sensor readings required to trigger skip mode.
    """

    if acceleration_speed is None:
        acceleration_speed = speed

    if deceleration_speed is None:
        deceleration_speed = speed

    # -------------------------------------------------
    # STOP TARGET
    # -------------------------------------------------

    junction_target = None

    if isinstance(distance, str):

        d = distance.lower()

        if d == "t":
            junction_target = "T"

        elif d == "<":
            junction_target = "<"

        elif d == ">":
            junction_target = ">"

    stop_at_junction = junction_target is not None

    if not stop_at_junction:

        target_distance = float(distance)

        if target_distance <= 0:
            lmotor.brake()
            rmotor.brake()
            return


    # Starting motor positions
    left_start = lmotor.angle()
    right_start = rmotor.angle()

    junction_count = 0

    # Skip state
    skipping = False
    skip_start = 0

    # Previous line-position error
    previous_pos = None


    while True:

        # Get line sensor data
        pos, der, shape_byte = await pr.call_multitask("lines")

        shape = chr(shape_byte)

        pos = pos + bias


        # Distance travelled since beginning
        travelled = distance_travelled(
            left_start,
            right_start
        )


        # -------------------------------------------------
        # STOP CONDITIONS
        # -------------------------------------------------

        if stop_at_junction:

            if shape == junction_target:
                junction_count += 1
            else:
                junction_count = 0

            if junction_count >= T_CONFIRM:
                break

        else:

            if travelled >= target_distance:
                break


        # -------------------------------------------------
        # FINISH CURRENT SKIP
        # -------------------------------------------------

        if skipping:

            if travelled - skip_start >= skip_distance:
                skipping = False

                # Reset reference so leaving skip mode
                # doesn't immediately trigger another skip.
                previous_pos = pos


        # -------------------------------------------------
        # START A SKIP FROM SUDDEN ERROR JUMP
        # -------------------------------------------------

        if (
            skip
            and not skipping
            and previous_pos is not None
        ):

            error_jump = abs(pos - previous_pos)

            if error_jump >= skip_error_jump:
                skipping = True
                skip_start = travelled


        # Store current error for next sensor reading
        previous_pos = pos


        # -------------------------------------------------
        # SPEED PHASE
        # -------------------------------------------------

        current_speed = speed

        # Initial slower section
        if (
            acceleration_distance > 0
            and travelled < acceleration_distance
        ):
            current_speed = acceleration_speed

        # After travelling deceleration_distance,
        # switch to deceleration_speed.
        elif (
            deceleration_distance > 0
            and travelled >= deceleration_distance
        ):
            current_speed = deceleration_speed


        # -------------------------------------------------
        # PID VALUES
        # -------------------------------------------------

        factor = current_speed / 30



        # -------------------------------------------------
        # SKIP MODE
        # -------------------------------------------------

        if skipping:

            # Ignore the sudden line-position error and
            # commit to driving straight.
            lmotor.dc(current_speed)
            rmotor.dc(current_speed)


        # -------------------------------------------------
        # NORMAL LINE FOLLOWING
        # -------------------------------------------------

        elif shape in "|<>TY":

            lpwr = (
                current_speed
                - abs(der) * D_BRAKE
                - pos * kp
                - der * kd
            )

            rpwr = (
                current_speed
                - abs(der) * D_BRAKE
                + pos * kp
                + der * kd
            )

            lpwr = max(-100, min(100, lpwr))
            rpwr = max(-100, min(100, rpwr))

            lmotor.dc(lpwr)
            rmotor.dc(rpwr)

        else:

            lmotor.dc(current_speed)
            rmotor.dc(current_speed)


        await wait(5)


    lmotor.brake()
    rmotor.brake()
from pybricks.tools import wait
from math import sqrt


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


async def micro_straight(
    distance,
    target_heading,
    speed=70,
    kp=3,
    max_turn=15,
    tolerance=1
):
    start = db.distance()
    direction = 1 if distance > 0 else -1
    target_distance = abs(distance)

    while True:
        travelled = abs(db.distance() - start)
        remaining = target_distance - travelled

        if remaining <= tolerance:
            break

        error = target_heading - db.angle()

        turn_rate = kp * error
        turn_rate = max(-max_turn, min(max_turn, turn_rate))

        db.drive(
            direction * speed,
            turn_rate
        )

        await wait(10)

    db.hold()
async def micro_turn_to(target, tol=0.5):
    while True:
        error = target - db.angle()

        if abs(error) <= tol:
            break

        rate = error * 4

        if rate > 20:
            rate = 20
        elif rate < -20:
            rate = -20

        if 0 < rate < 8:
            rate = 8
        elif -8 < rate < 0:
            rate = -8

        db.drive(0, rate)
        await wait(5)

    db.hold()

async def pull():
    global matrix
    global scan_id

    # --------------------------------------------------
    # SPIKE OWNS THE REQUEST ID
    # --------------------------------------------------

    scan_id += 1

    if scan_id > 255:
        scan_id = 1

    requested_id = scan_id

    print(
        "Requesting scan:",
        requested_id
    )


    # --------------------------------------------------
    # TOTAL TIMEOUT
    # --------------------------------------------------

    timer = StopWatch()

    # Track when we last sent the request
    last_send = -1000

    # Count attempts for debugging
    attempts = 0

    # Don't print the same stale cam ID repeatedly
    last_printed_id = -1


    # --------------------------------------------------
    # WAIT FOR THE CORRECT MATRIX
    #
    # Important:
    # We resend the SAME request ID if necessary.
    # --------------------------------------------------

    while True:

        now = timer.time()


        # ----------------------------------------------
        # SEND / RESEND SCAN REQUEST
        #
        # 200 ms between retries.
        # SAME request ID every time.
        # ----------------------------------------------

        if now - last_send >= 200:

            attempts += 1

            print(
                "Sending scan request:",
                requested_id,
                "attempt:",
                attempts
            )

            await pr.call_multitask(
                "scan",
                requested_id,
                wait_ms=40
            )

            last_send = timer.time()


        # ----------------------------------------------
        # Give process_async() time to work
        # ----------------------------------------------

        await wait(20)


        # ----------------------------------------------
        # READ CAMERA CACHE
        # ----------------------------------------------

        data = await pr.call_multitask(
            "cam"
        )

        received_id = data[0]


        # Diagnostic only when value changes
        if received_id != last_printed_id:

            print(
                "Camera cache ID:",
                received_id
            )

            last_printed_id = received_id


        # ----------------------------------------------
        # CORRECT RESULT RECEIVED
        # ----------------------------------------------

        if received_id == requested_id:
            break


        # ----------------------------------------------
        # TIMEOUT
        # ----------------------------------------------

        if timer.time() > 3000:

            print(
                "WARNING: camera timeout"
            )

            print(
                "Requested:",
                requested_id
            )

            print(
                "Last camera ID:",
                received_id
            )

            print(
                "Attempts:",
                attempts
            )

            print(
                "Using fallback matrix"
            )


            # ----------------------------------------------
            # FALLBACK MATRIX
            #
            # Left half  = BLUE   = 2
            # Right half = YELLOW = 1
            #
            # 2 2 1 1
            # 2 2 1 1
            # 2 2 1 1
            # ----------------------------------------------

            data = [
                requested_id,

                2, 2, 1, 1,
                2, 2, 1, 1,
                2, 2, 1, 1
            ]

            # Make the rest of pull() behave exactly as if
            # this request ID had returned successfully.
            received_id = requested_id

            break

    # ==================================================
    # DISPLAY MATRIX
    # ==================================================

    matrix = data[1:]


    print()
    print(
        "SCAN RECEIVED:",
        received_id
    )

    print(
        "Attempts:",
        attempts
    )

    print()

    print(
        matrix[0],
        matrix[1],
        matrix[2],
        matrix[3]
    )

    print(
        matrix[4],
        matrix[5],
        matrix[6],
        matrix[7]
    )

    print(
        matrix[8],
        matrix[9],
        matrix[10],
        matrix[11]
    )

    print()

    print("SUCCESS")

async def toscan():
    global result
    db.settings(straight_speed=350)
    db.use_gyro(True)
    await db.arc(60,77, then=Stop.NONE)
    await multitask(line(90, 650, acceleration_distance=80, skip=True, bias=20), claw.run_angle(500,160), stopper.run_target(500, 45))
    await line(90, "t", bias=20)
    await db.arc(-60,90)
    await line(35, "t")

    global matrix
    await wait(100)
    await pull()
    await wait(600)
    matrix = make_matrix(matrix)
    matrix = rotate_matrix(matrix, 270)    
    result = plan_blocks(matrix, start_station=1)

    print("Sequence:", result["plan"])
    print("Distance:", result["distance"])

async def tools_sub():
    await slide.run_angle(1000,250)
    await wait(100)
    await slide.run_angle(-800,250)
async def release_sub():
    await wait(1530)
    await slide.run_angle(1000,370)
    await wait(250)
    await slide.run_angle(1000,-370)
async def tools():
    db.settings(straight_speed=120)
    await multitask(tools_sub(), db.straight(-155))
    db.settings(straight_speed=500)

    await db.turn(83)
    await db.straight(-420)
    await db.arc(-1000,25)
    await db.turn(35)
    await line(100, 420, acceleration_distance=70, acceleration_speed=50, skip=True, bias=20)
    await line(50, "t", bias=20)
    await wait(100)
    await db.turn(70)
    await multitask(db.straight(-140),slide.run_angle(1000,300))
    await multitask(db.straight(100), slide.run_angle(1000,-50))
    await db.arc(135,90)
    await db.straight(50)
    await multitask(db.arc(70,30),slide.run_angle(1000,-100))
    await db.straight(450)
    await db.turn(-10)
    db.settings(straight_speed=400)
    await multitask(db.straight(790), slide.run_angle(1000,-150))

    await multitask(db.arc(-100, 30))
    await multitask(db.straight(-67), claw.run_angle(800, 0))
    await db.arc(-36,-119, then=Stop.COAST_SMART)


    db.drive(-150, 0)
    await wait(600)
    db.brake()
    await multitask(db.straight(330))
    await wait(100)


# Persistent state

block_pos = {
    1: 0,   # Yellow
    2: 0,   # Blue
    3: 0,   # Green
    4: 0    # White
}

COLPOS = {
    1: 0,
    2: 420,
    3: 840
}

RCOLLECTPOS = 0
LCOLLECTPOS = 700

slidepos = RCOLLECTPOS

station_dist = 164
station_entry_dist = 74
rowdist = 62

claw_lift = 90
claw_drop = 75
current_station = 1
def block_side(pos):
    # 0 R, 1 L, 2 R, 3 L, 4 R, 5 L
    return "r" if pos % 2 == 0 else "l"


def block_row(pos):
    return pos // 2


def collect_position(side):
    return RCOLLECTPOS if side == "r" else LCOLLECTPOS


async def slide_to(target):
    global slidepos

    if target != slidepos:
        await slide.run_angle(
            1000,
            target - slidepos
        )
        slidepos = target


async def slide_to_side(side):
    await slide_to(
        collect_position(side)
    )


async def deposit_block(chamber):
    if chamber not in (1, 2, 3):
        raise ValueError("Invalid chamber")

    await slide_to(
        COLPOS[chamber]
    )

    claw.control.limits(acceleration=5000)
    await claw.run_angle(
        5000,
        -claw_drop,
        then=Stop.HOLD
    )


async def normal_block(chamber, side, next_side):
    claw.control.limits(acceleration=600)

    await claw.run_angle(
        380,
        -claw_lift,
        then=Stop.HOLD
    )

    await deposit_block(chamber)

    if side == "r" and next_side == "l":

        await multitask(
            slide_to_side("l"),
            db.straight(-rowdist),
            claw.run_angle(
                170,
                claw_lift+claw_drop,
                then=Stop.HOLD
            )
        )
        db.settings(straight_speed=120, straight_acceleration=800)
        await db.straight(rowdist)

        return 0

    elif side == "l" and next_side == "r":

        await multitask(
            slide_to_side("r"),
            claw.run_angle(
                300,
                claw_lift+claw_drop,
                then=Stop.HOLD
            )
        )
        db.settings(straight_speed=120, straight_acceleration=800)

        await db.straight(rowdist)

        return 1

    raise ValueError("Unexpected block sequence")


async def final_block_mechanism(
    chamber,
    next_station
):
    global slidepos

    claw.control.limits(acceleration=600)

    await claw.run_angle(
        380,
        -claw_lift,
        then=Stop.HOLD
    )

    await deposit_block(chamber)

    if next_station is None:
        return

    next_pos = block_pos[next_station]
    next_side = block_side(next_pos)
    target_slide = collect_position(next_side)

    slide_difference = (
        target_slide - slidepos
    )

    if slide_difference != 0:

        await multitask(
            slide.run_angle(
                1000,
                slide_difference
            ),

            claw.run_angle(
                300,
                claw_drop+claw_lift,
                then=Stop.HOLD
            )
        )

        slidepos = target_slide

    else:

        await claw.run_angle(
            300,
            claw_lift+claw_drop,
            then=Stop.HOLD
        )


async def exit_and_transition(
    station,
    current_row,
    next_station
):
    exit_distance = (
        station_entry_dist
        + current_row * rowdist
    )
    await wait(800)
    await db.straight(
        -exit_distance
    )

    await db.turn(90)

    if next_station is None:
        return

    station_change = (
        next_station - station
    )

    db.settings(
        straight_speed=350,
        straight_acceleration=900,
        turn_acceleration=(400, 250),
        turn_rate=200
    )

    if station_change != 0:
        await db.straight(
            station_change * station_dist
        )

    db.settings(
        straight_speed=200,
        straight_acceleration=700,
        turn_acceleration=(500, 300),
        turn_rate=200
    )

    await db.turn(-90)


async def finish_station(
    station,
    chamber,
    current_row,
    next_station
):
    await multitask(
        final_block_mechanism(
            chamber,
            next_station
        ),

        exit_and_transition(
            station,
            current_row,
            next_station
        )
    )


async def collect_station(
    station,
    chambers,
    next_station,
    backup_before_line=False
):
    if station not in (1, 2, 3, 4):
        raise ValueError("Invalid station")

    if len(chambers) == 0:
        return

    start_pos = block_pos[station]

    if start_pos + len(chambers) > 6:
        raise ValueError(
            "Not enough blocks left at station"
        )

    # Slide should already be prepared for
    # this station by the previous transition.
    if backup_before_line:
        await db.straight(-50)

    await line(35, "t", kp=-1.22, kd=-0.01)
    await db.straight(-5)
    db.reset(angle=0)
    await wait(50)

    first_row = block_row(start_pos)

    current_row = 0
    db.settings(straight_speed=120, straight_acceleration=800)
    if first_row > 0:
        await db.straight(
            first_row * rowdist
        )

        current_row = first_row


    for index, chamber in enumerate(chambers):

        pos = start_pos + index
        side = block_side(pos)

        is_last = (
            index == len(chambers) - 1
        )

        if is_last:

            # Update before finish_station so that
            # revisiting this same color works.
            block_pos[station] = pos + 1

            await finish_station(
                station,
                chamber,
                current_row,
                next_station
            )

            return


        next_pos = pos + 1
        next_side = block_side(next_pos)

        rows_advanced = await normal_block(
            chamber,
            side,
            next_side
        )

        current_row += rows_advanced

        block_pos[station] = pos + 1


async def collect(ins):
    global current_station
    if not ins:
        return

    # Robot starts:
    # station 1
    # facing along station lane
    # slide at RIGHT collection position
    # claw down

    first_station = ins[0][0]

    station_change = (
        first_station - current_station
    )

    db.settings(
        straight_speed=350,
        straight_acceleration=900,
        turn_acceleration=(400, 250),
        turn_rate=200
    )

    if station_change != 0:
        await db.straight(
            station_change * station_dist
        )
    current_station = first_station
    db.settings(
        straight_speed=200,
        straight_acceleration=700,
        turn_acceleration=(500, 300),
        turn_rate=200
    )

    await db.turn(-92.5)

    # Prepare for first block
    first_pos = block_pos[first_station]

    await slide_to_side(
        block_side(first_pos)
    )


    for index, (station, chambers) in enumerate(ins):

        if index + 1 < len(ins):
            next_station = ins[index + 1][0]
        else:
            next_station = None

        # Back up before line() if:
        # 1. this is the first collection station
        # 2. previous station change was more than 2 stations
        if index == 0:
            backup_before_line = True
        else:
            previous_station = ins[index - 1][0]
            station_change = station - previous_station

            backup_before_line = abs(station_change) > 2

        await collect_station(
            station,
            chambers,
            next_station,
            backup_before_line
        )

        if next_station is not None:
            current_station = next_station
        else:
            current_station = station
async def collect_to_deposit():
    db.settings(straight_speed=300)
    print(current_station)
    dist = 0
    if current_station == 1:
        dist = 230
    elif current_station == 2:
        dist = 82
    elif current_station == 3:
        dist = -82
    elif current_station == 4:
        dist = -250
    await db.straight(dist)
    await multitask(db.turn(-89),claw.run_angle(1000, 50))
    await multitask(db.straight(-270),slide.run_angle(1000, 0-slidepos))
    await line(35, "t", kp=-1.6, kd=-0.002)
    await wait(100)
    db.reset(angle=0)
async def deposit():
    await line(35, "t", kp=-1.6, kd=-0.002)
    await wait(100)
    db.reset(angle=0)
    db.settings(straight_speed=300)
    db.use_gyro(True)
    await multitask(db.straight(-536),slide.run_angle(1000, 400))

    await stopper.run_target(500, 45)
    db.settings(straight_speed=500)

    stopper.control.limits(acceleration=5000)
    #await stopper.run_target(800, 75)
    await stopper.run_target(400, -35)
    await wait(500)
    await stopper.run_target(800,45)
    await wait(300)
    await db.straight(-40)
    await db.straight(70)
    await wait(400)
    #2
    await stopper.run_target(800, 65)
    await stopper.run_target(400, -35)
    await wait(500)
    await stopper.run_target(800,45)
    await wait(300)
    await db.straight(-25)
    await db.straight(83)
    await wait(400)
    
    await stopper.run_target(800, 65)
    await stopper.run_target(300, -35)
    await wait(500)
    await stopper.run_target(800,45)
    await wait(100)
    await db.straight(-30)
    await db.straight(82)
    await wait(200)

    await stopper.run_target(300, -35)
    await stopper.run_target(500, 10)
    await stopper.run_target(300, -35)
    await wait(500)
    await db.straight(-25)
    await db.straight(78)
    await wait(200)


async def test():
    db.use_gyro(True)
    await line(35, "t", kp=-1.52, kd=-0.01)
async def slide_test():
    slide.control.limits(acceleration=10000)
    a = 800
    for i in range(4):
        await slide.run_angle(1200,a)

        await slide.run_angle(1000,-a)

async def test2():

    #await collect_to_deposit()
    await deposit()
async def main():
    
    await toscan()
    await tools()
    await collect(result["plan"])
    await collect_to_deposit()
    await deposit()

print("Voltage:", hub.battery.voltage())
run_task(multitask(pr.process_async(),main()))