import dstar2
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

ROWS = 5  # Along x-axis, OGM Width
COLS = 5  # Along y-axis, OGM Height

GRID_XSIZE = WINDOW_WIDTH // ROWS
GRID_YSIZE = WINDOW_HEIGHT // COLS


def draw_ogm(surface, map: dstar2.OGM):
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
            for p in path:  # (y,x)
                center = [(p[1] * GRID_XSIZE + GRID_XSIZE / 2), (p[0] * GRID_YSIZE + GRID_YSIZE / 2)]
                pygame.draw.circle(surface, RED, center, min(GRID_XSIZE, GRID_YSIZE) / 2.0)


def get_grid_pos(surface, pos, rows=ROWS, cols=COLS):
    width, height = surface.get_size()
    return int(pos[1] // (height / rows)), int(pos[0] // (width / cols))  # y, x


if __name__ == '__main__':
    planner = dstar2.DStar()
    planner.initialize((0, 0), (ROWS-1, COLS-1))

    map = dstar2.OGM(ROWS, COLS)
    for y in range(0, ROWS, 4):
        if y + 1 < ROWS:
            for x in range(0, COLS - 1):
                map.set_obstacle((y + 1, x))
        if y + 3 < ROWS:
            for x in range(1, COLS):
                map.set_obstacle((y + 3, x))

    planner.update_map(map)
    changed_cells = {}
    path = []
    if planner.re_plan():
        path = planner.get_path().path

    print("Path:", path)

    pygame.init()
    pygame.display.set_caption("DStar")
    main_window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    running = True

    if len(path):
        pygame.display.set_caption(f"DStar: Path length {len(path)}")
    # print(planner.map.get_predecessors((3, 0)))
    # print(planner.map.get_predecessors((3, 1)))
    # print(planner.map.get_successors((3, 0)))
    # print(planner.map.get_successors((3, 1)))

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
                        planner.map.set_obstacle(pos)
                        planner.update_map_node(pos, planner.map.OBSTACLE)

                elif event.button == pygame.BUTTON_RIGHT:  # Right-button is pressed, change to free if obstacle
                    pos = pygame.mouse.get_pos()  # (col, row)
                    pos = get_grid_pos(main_window, pos)  # x, y
                    if planner.map.is_obstacle(pos):
                        planner.map.set_free(pos)
                        planner.update_map_node(pos, planner.map.FREE)

            elif event.type == pygame.MOUSEBUTTONUP:  # Mouse button is released
                if len(path) == 0:
                    continue

                if planner.re_plan():
                    path = planner.get_path().path
                    pygame.display.set_caption(f"Planner: Path length {len(path)}")
                else:
                    pygame.display.set_caption(f"Planner: Path not found!!!")

                # print(modified_cells)
                changed_cells.clear()  # Clear stored cells list, during mouse being pressed

        pygame.display.update()
        # print(f"Time [mSec]: {pygame.time.get_ticks()}")
        pygame.time.wait(1)  # 50 Hz

    pygame.quit()
