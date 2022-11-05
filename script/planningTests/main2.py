from dstar2 import DStar, Path, OGM, OBSTACLE, FREE, INF
import time
import pygame

BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)

BACKGROUND = WHITE
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

ROWS = 10  # Along x-axis, OGM Width
COLS = 10  # Along y-axis, OGM Height

GRID_XSIZE = WINDOW_WIDTH // ROWS
GRID_YSIZE = WINDOW_HEIGHT // COLS


def draw_ogm(surface, map: OGM):
    row, col = 0, 0
    for x in range(0, WINDOW_WIDTH, GRID_XSIZE):
        for y in range(0, WINDOW_HEIGHT, GRID_YSIZE):
            rect = pygame.Rect(x, y, GRID_XSIZE, GRID_YSIZE)
            if map[(row, col)] == FREE:
                pygame.draw.rect(surface, WHITE, rect, 1)
            else:
                pygame.draw.rect(surface, BLACK, rect)
            row += 1
        row = 0
        col += 1
    return map

def draw_path(surface, path: Path):
    if path is not None:
        if len(path.pos) > 1:
            center = [(path.pos[0][1] * GRID_XSIZE + GRID_XSIZE / 2), (path.pos[0][0] * GRID_YSIZE + GRID_YSIZE / 2)]
            pygame.draw.circle(surface, BLUE, center, min(GRID_XSIZE, GRID_YSIZE) / 2.0)
            for p in path.pos[1:]:  # (y,x)
                center = [(p[1] * GRID_XSIZE + GRID_XSIZE / 2), (p[0] * GRID_YSIZE + GRID_YSIZE / 2)]
                pygame.draw.circle(surface, RED, center, min(GRID_XSIZE, GRID_YSIZE) / 2.0)


def get_grid_pos(surface, pos, rows=ROWS, cols=COLS):
    width, height = surface.get_size()
    return int(pos[1] // (height / rows)), int(pos[0] // (width / cols))  # y, x


if __name__ == '__main__':
    pygame.init()
    pygame.display.set_caption("DStar")
    main_window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    running = True

    map = OGM(ROWS, COLS)
    for y in range(0, ROWS, 4):
        if y + 1 < ROWS:
            for x in range(0, COLS - 1):
                map.set_obstacle((y + 1, x))
        if y + 3 < ROWS:
            for x in range(1, COLS):
                map.set_obstacle((y + 3, x))


    planner = DStar()
    planner.initialize((0, 0), (ROWS-1, COLS-1))
    planner.update_map(map)
    planner.re_plan()
    path = planner.get_path()
    print("Path:", path.pos)
    pygame.display.set_caption(f"DStar: Path length {path.path_length:0.2f}")

    while running:
        main_window.fill(BACKGROUND)
        map.map = draw_ogm(main_window, planner.get_map())
        draw_path(main_window, path)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # Exit program
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:  # Exit program
                    running = False

                elif event.key == pygame.K_SPACE:  # Move to next path location
                    if len(path.pos) > 1:
                        path.pos.pop(0)
                        planner.update_start(path.pos[0])
                        if planner.re_plan():
                            path = planner.get_path()
                            pygame.display.set_caption(f"Planner: Path length {path.path_length:0.2f}")
                        else:
                            pygame.display.set_caption(f"Planner: Path not found!!!")

                elif event.key == pygame.K_r:  # Move to next path location
                    # planner.update_start((0,0))
                    planner.initialize(path.pos[0], (ROWS-1, COLS-1))
                    planner.update_map(map)
                    planner.re_plan()
                    path = planner.get_path()

            # elif pygame.mouse.get_pressed()[0]:  # Left mouse button is being pressed
            #
            # elif pygame.mouse.get_pressed()[2]:  # Right mouse button is being pressed

            elif event.type == pygame.MOUSEBUTTONDOWN:  # Mouse button is pressed, change to obstacle if free
                if event.button == pygame.BUTTON_LEFT:  # Left-button is pressed
                    pos = pygame.mouse.get_pos()  # (col, row)
                    pos = get_grid_pos(main_window, pos)  # x, y
                    planner.update_map_node(pos, OBSTACLE)
                    for n_pos, n_cost in map.get_neighbours(pos).items():
                        key = hash(n_pos)
                        if key in planner.LT:
                            planner.update_map_node(n_pos, planner.LT[key].cost)

                elif event.button == pygame.BUTTON_RIGHT:  # Right-button is pressed, change to free if obstacle
                    pos = pygame.mouse.get_pos()  # (col, row)
                    pos = get_grid_pos(main_window, pos)  # x, y
                    planner.update_map_node(pos, FREE)
                    for n_pos, n_cost in map.get_neighbours(pos).items():
                        # planner.update_map_node(n_pos, FREE)
                        key = hash(n_pos)
                        if key in planner.LT:
                            planner.update_map_node(n_pos, planner.LT[key].cost)

            elif event.type == pygame.MOUSEBUTTONUP:  # Mouse button is released
                    planner.update_start(planner.start.pos)

                    if planner.re_plan():
                        path = planner.get_path()
                        pygame.display.set_caption(f"Planner: Path length {path.path_length:0.2f}")
                        print(f"{path.path_length}: {path.pos}")
                    else:
                        pygame.display.set_caption(f"Planner: Path not found!!!")

        pygame.display.update()
        # print(f"Time [mSec]: {pygame.time.get_ticks()}")
        pygame.time.wait(1)  # 50 Hz

    pygame.quit()
