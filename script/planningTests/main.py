from astar import AStar, QNode, OGM
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
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 1400

ROWS = 40  # Along x-axis, OGM Width
COLS = 40  # Along y-axis, OGM Height

GRID_XSIZE = WINDOW_WIDTH // ROWS
GRID_YSIZE = WINDOW_HEIGHT // COLS


def draw_ogm(surface, map: OGM):
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
    pygame.init()
    pygame.display.set_caption("DStar")
    main_window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    running = True
    m = OGM(ROWS, COLS)
    m.set_obstacle((5, 5))
    astar = AStar()
    astar.set_map(m)
    astar.set_start(QNode((0, 0)))
    astar.set_goal(QNode((ROWS-1, COLS-1)))
    path = []
    if astar.search_path():
        path = astar.get_path()
        pygame.display.set_caption(f"DStar: Path length {len(path)}")

    while running:
        main_window.fill(BACKGROUND)
        draw_ogm(main_window, astar.map)
        draw_path(main_window, path)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    if astar.search_path():
                        path = astar.get_path()
                        pygame.display.set_caption(f"DStar: Path length {len(path)}")
                    else:
                        path = []
                        pygame.display.set_caption(f"DStar: Path not found!!!")
            elif pygame.mouse.get_pressed()[0]:  # LEFT BUTTON DOWN
                pos = pygame.mouse.get_pos()  # (col, row)
                astar.map.set_obstacle(get_grid_pos(main_window, pos))
            elif pygame.mouse.get_pressed()[2]:  # RIGHT BUTTON DOWN
                pos = pygame.mouse.get_pos()  # (col, row)
                astar.map.set_free(get_grid_pos(main_window, pos))
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == pygame.BUTTON_MIDDLE:
                    if astar.search_path():
                        path = astar.get_path()
                        pygame.display.set_caption(f"DStar: Path length {len(path)}")
                    else:
                        path = []
                        pygame.display.set_caption(f"DStar: Path not found!!!")


        # pygame.display.update()
        pygame.display.flip()
        # print(f"Time [mSec]: {pygame.time.get_ticks()}")
        pygame.time.wait(1)  # 50 Hz

    pygame.quit()
