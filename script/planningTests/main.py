import astar
import dstar
import time
import pygame


def list_dict_lookup_performance():
    import time
    ll = [i for i in range(100_000_000)]
    dd = {i: i for i in range(100_000_000)}

    start = time.time()
    print(99_999_999 in dd)
    print(f"DD Exec. time: {time.time() - start}")

    start = time.time()
    print(99_999_999 in ll)
    print(f"LL Exec. time: {time.time() - start}")


BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)

BACKGROUND = WHITE
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

ROWS = 5  # Along x-axis, OGM Width
COLS = 5  # Along y-axis, OGM Height

GRID_XSIZE = WINDOW_WIDTH // ROWS
GRID_YSIZE = WINDOW_HEIGHT // COLS


def draw_ogm(surface, map: dstar.OGM):
    row, col = 0, 0
    for x in range(0, WINDOW_WIDTH, GRID_XSIZE):
        for y in range(0, WINDOW_HEIGHT, GRID_YSIZE):
            rect = pygame.Rect(x, y, GRID_XSIZE, GRID_YSIZE)
            if map.is_free((row, col)):
                pygame.draw.rect(surface, WHITE, rect, 1)
            else:
                pygame.draw.rect(surface, BLACK, rect)
            row += 1
        row = 0
        col += 1


def draw_path(surface, path):
    if path is not None:
        if len(path):
            for p in path: # (y,x)
                center = [(p[1]*GRID_XSIZE + GRID_XSIZE/2), (p[0]*GRID_YSIZE + GRID_YSIZE/2)]
                circ = pygame.draw.circle(surface, RED, center, min(GRID_XSIZE, GRID_YSIZE)/2.0)


def get_grid_pos(surface, pos, rows=ROWS, cols=COLS):
    width, height = surface.get_size()
    return int(pos[1] // (height / rows)), int(pos[0] // (width / cols))  # y, x


if __name__ == '__main__':
    # list_dict_lookup_performance()

    ### DStar planner ###
    planner = dstar.DStar()
    planner.set_map(dstar.OGM(ROWS, COLS))
    planner.set_start(dstar.QNode((0, 0), (float('inf'), float('inf'))))
    planner.set_goal(dstar.QNode((ROWS-1, COLS-1), (float('inf'), float('inf'))))
    for y in range(0, ROWS, 4):
        if y + 1 < ROWS:
            for x in range(0, COLS - 1):
                planner.map.set_obstacle((y + 1, x))
        if y + 3 < ROWS:
            for x in range(1, COLS):
                planner.map.set_obstacle((y + 3, x))

    planner.initialize()
    changed_cells = {}
    path = planner.re_plan(planner.start.pos, changed_cells)
    print("Path:", path)

    ### AStar planner ###
    # planner = astar.AStar()
    # planner.set_map(astar.OGM(ROWS, COLS))
    # planner.set_start(astar.QNode((0, 0), float('inf')))
    # planner.set_goal(astar.QNode((ROWS-1, COLS-1), float('inf')))

    pygame.init()
    pygame.display.set_caption("DStar")
    main_window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    running = True

    if len(path):
        pygame.display.set_caption(f"DStar: Path length {len(path)}")

    print(planner.map.get_predecessors((3,0)))
    print(planner.map.get_predecessors((3,1)))

    while running:
        main_window.fill(BACKGROUND)
        draw_ogm(main_window, planner.map)
        draw_path(main_window, path)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # Exit program
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:  # Exit program
                    running = False

                if event.key == pygame.K_SPACE:  # Move to next path location
                    if len(path) > 1:
                        path.pop(0)
                        _path = planner.re_plan(path[0], changed_cells)  # use current location and stored cells
                        if len(_path):
                            path = _path
                            pygame.display.set_caption(f"Planner: Path length {len(path)}")
                        else:
                            pygame.display.set_caption(f"Planner: Path not found!!!")

            # elif pygame.mouse.get_pressed()[0]:  # Left mouse button is being pressed
            #
            # elif pygame.mouse.get_pressed()[2]:  # Right mouse button is being pressed

            elif event.type == pygame.MOUSEBUTTONDOWN:  # Mouse button is pressed, change to obstacle if free
                if event.button == pygame.BUTTON_LEFT:  # Left-button is pressed
                    pos = pygame.mouse.get_pos()  # (col, row)
                    pos = get_grid_pos(main_window, pos)  # x, y
                    if planner.map.is_free(pos):
                        neighbors_with_old_cost = planner.map.get_successors(pos)
                        changed_cells[pos] = neighbors_with_old_cost
                        planner.map.set_obstacle(pos)

                elif event.button == pygame.BUTTON_RIGHT:  # Right-button is pressed, change to free if obstacle
                    pos = pygame.mouse.get_pos()  # (col, row)
                    pos = get_grid_pos(main_window, pos)  # x, y
                    if planner.map.is_obstacle(pos):
                        neighbors_with_old_cost = planner.map.get_successors(pos)
                        changed_cells[pos] = neighbors_with_old_cost
                        planner.map.set_free(pos)

            elif event.type == pygame.MOUSEBUTTONUP:  # Mouse button is released
                # replan
                if len(path) == 0:
                    continue

                _path = planner.re_plan(path[0], changed_cells)  # use current location and stored cells
                if len(_path):
                    path = _path
                    pygame.display.set_caption(f"Planner: Path length {len(path)}")
                else:
                    pygame.display.set_caption(f"Planner: Path not found!!!")

                # print(modified_cells)
                changed_cells.clear() # Clear stored cells list, during mouse being pressed



        pygame.display.update()
        # print(f"Time [mSec]: {pygame.time.get_ticks()}")
        pygame.time.wait(1)  # 50 Hz

    pygame.quit()
