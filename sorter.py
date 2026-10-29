# Sorting program that processes scanned colors and returns a collection sequence

# Color / station numbers
YELLOW = 1
BLUE   = 2
GREEN  = 3
WHITE  = 4

def make_matrix(m):
    row1 = []
    row2 = []
    row3 = []
    count = 1
    for i in m:
        if count <= 4:
            row1.append(i)
        elif count > 4 and count <=8:
            row2.append(i)
        elif count > 8:
            row3.append(i)
        count += 1
    
    made_matrix = [row1,row2,row3]
    return made_matrix



def rotate_matrix(matrix, degrees):
    if degrees == 90:
        return [list(row) for row in zip(*matrix[::-1])]

    elif degrees == 180:
        return [row[::-1] for row in matrix[::-1]]

    elif degrees == 270:
        return [list(row) for row in zip(*matrix)][::-1]

    else:
        raise ValueError("degrees must be 90, 180, or 270")

def plan_blocks(matrix, start_station=1):
    """
    Find the minimum-distance collection plan for a 4x3 mosaic.

    matrix:
        4 rows x 3 columns.

        Example:
        [
            [1, 2, 3],
            [2, 2, 4],
            [3, 1, 4],
            [4, 3, 1]
        ]

        Each column corresponds to one storage chamber.
        Blocks must enter each chamber top-to-bottom because
        the chambers are FIFO.

    start_station:
        Robot's starting color station.
        1 = yellow
        2 = blue
        3 = green
        4 = white

    Returns:
        {
            "plan": [
                (station, [chamber, chamber, ...]),
                ...
            ],

            "distance": total station-to-station travel,

            "chambers": [
                [...],
                [...],
                [...]
            ]
        }
    """

    # -----------------------------
    # Validate the matrix
    # -----------------------------

    if len(matrix) != 4:
        raise ValueError("Matrix must contain 4 rows.")

    for row in matrix:
        if len(row) != 3:
            raise ValueError("Each row must contain 3 columns.")

        for color in row:
            if color not in (1, 2, 3, 4):
                raise ValueError(
                    "Colors must be 1, 2, 3, or 4."
                )

    if start_station not in (1, 2, 3, 4):
        raise ValueError(
            "start_station must be 1, 2, 3, or 4."
        )

    # -----------------------------
    # Convert matrix columns into
    # FIFO chamber queues
    # -----------------------------

    chambers = [
        [matrix[row][0] for row in range(4)],
        [matrix[row][1] for row in range(4)],
        [matrix[row][2] for row in range(4)]
    ]

    # State:
    #
    # (p1, p2, p3, station)
    #
    # p1/p2/p3 = number of blocks already loaded
    # into each chamber.
    #
    # Example:
    # (2, 1, 3, 2)
    #
    # chamber 1 has 2 blocks
    # chamber 2 has 1 block
    # chamber 3 has 3 blocks
    # robot is at station 2

    memo = {}

    def solve(pointers, current_station):

        state = (
            pointers[0],
            pointers[1],
            pointers[2],
            current_station
        )

        if state in memo:
            return memo[state]

        # All 12 blocks collected
        if pointers == (4, 4, 4):
            result = (0, [])
            memo[state] = result
            return result

        # --------------------------------
        # Find which colors are currently
        # legal to collect
        # --------------------------------

        available_stations = []

        for chamber_index in range(3):

            pointer = pointers[chamber_index]

            if pointer < 4:

                color = chambers[chamber_index][pointer]

                if color not in available_stations:
                    available_stations.append(color)

        best_cost = None
        best_plan = None

        # --------------------------------
        # Try every legal next station
        # --------------------------------

        for station in available_stations:

            new_pointers = list(pointers)

            chamber_actions = []

            # --------------------------------
            # Once at a station, collect every
            # block that can legally be taken
            # from that station.
            #
            # Also handles consecutive blocks
            # of the same color in one chamber.
            # --------------------------------

            for chamber_index in range(3):

                while new_pointers[chamber_index] < 4:

                    next_color = chambers[chamber_index][
                        new_pointers[chamber_index]
                    ]

                    if next_color != station:
                        break

                    # Chamber numbers returned as 1,2,3
                    chamber_actions.append(
                        chamber_index + 1
                    )

                    new_pointers[chamber_index] += 1

            new_pointers = tuple(new_pointers)

            # Physical travel between color stations
            move_cost = abs(
                station - current_station
            )

            future_cost, future_plan = solve(
                new_pointers,
                station
            )

            total_cost = move_cost + future_cost

            if (
                best_cost is None
                or total_cost < best_cost
            ):
                best_cost = total_cost

                best_plan = [
                    (station, chamber_actions)
                ] + future_plan

        result = (best_cost, best_plan)

        memo[state] = result

        return result

    # Start with all chambers empty
    distance, plan = solve(
        (0, 0, 0),
        start_station
    )

    return {
        "plan": plan,
        "distance": distance,
        "chambers": chambers
    }
