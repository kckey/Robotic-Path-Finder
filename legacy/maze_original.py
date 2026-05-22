import pygame, sys, heapq

# --------------------------------------------------------------------------- #
SCREEN_W = 800
SCREEN_H = 600
FPS      = 60
GRID_SIZE = 20

START_COLOR       = (80, 230, 150)
START_RING_COLOR  = (30, 150, 100)
END_COLOR         = (255, 120, 120)
END_RING_COLOR    = (200, 60, 80)
WALL_COLOR        = (60, 78, 115)
WALL_HILITE       = (80, 110, 150)
PATH_COLOR        = (165, 210, 255)
PATH_BORDER_COLOR = (70, 125, 190)
ROBOT_COLOR       = (40, 130, 255)
ROBOT_CORE_COLOR  = (255, 255, 255)
ROBOT_RING_COLOR  = (30, 70, 150)
TEXT_COLOR        = (30, 40, 70)
TEXT_MUTED        = (80, 90, 120)
GRID_COLOR        = (205, 212, 228)
GRID_ACCENT       = (185, 194, 220)

BG_TOP_COLOR    = (248, 250, 255)
BG_BOTTOM_COLOR = (212, 224, 246)
PANEL_TOP_COLOR = (255, 255, 255)
PANEL_BOTTOM_COLOR = (234, 240, 255)
PANEL_BORDER    = (170, 180, 205)
PANEL_SHADOW    = (20, 26, 60, 70)

BUTTON_TEXT_COLOR     = (245, 248, 255)
BUTTON_DISABLED_TEXT  = (200, 204, 215)
BUTTON_BORDER_COLOR   = (255, 255, 255)
RESET_BTN_TOP         = (118, 148, 240)
RESET_BTN_BOTTOM      = (90, 120, 210)
GO_BTN_TOP            = (255, 186, 110)
GO_BTN_BOTTOM         = (255, 138, 90)
BUTTON_SHADOW_COLOR   = (15, 20, 45, 120)

STATUS_WARN_BG = (255, 212, 212, 160)
STATUS_OK_BG   = (210, 238, 255, 170)

STEP_MS = 150  # robot step delay in milliseconds

# Instruction UI
PANEL_W = 250

# --------------------------------------------------------------------------- #
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Robot Path Simulator")
clock  = pygame.time.Clock()
font   = pygame.font.SysFont(None, 24)
font_big = pygame.font.SysFont(None, 28)

# UI rects (place them away from panel)
btn_rect = pygame.Rect(SCREEN_W - 120, 10, 110, 32)   # Reset
go_rect  = pygame.Rect(SCREEN_W - 120, 50, 110, 32)   # Go

def lerp_color(c1, c2, t):
    if len(c1) == 4:
        return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(4))
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))

def draw_vertical_gradient(surface, rect, top_color, bottom_color):
    height = rect.height
    if height <= 0:
        return
    for y in range(height):
        ratio = y / max(1, height - 1)
        color = lerp_color(top_color, bottom_color, ratio)
        pygame.draw.line(surface, color, (rect.left, rect.top + y), (rect.right - 1, rect.top + y))

def draw_rounded_gradient(surface, rect, top_color, bottom_color, radius=12):
    grad_surface = pygame.Surface(rect.size, pygame.SRCALPHA)
    draw_vertical_gradient(grad_surface, grad_surface.get_rect(), top_color, bottom_color)
    mask = pygame.Surface(rect.size, pygame.SRCALPHA)
    pygame.draw.rect(mask, (255, 255, 255), mask.get_rect(), border_radius=radius)
    grad_surface.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    surface.blit(grad_surface, rect.topleft)

def draw_shadow(rect, offset=(6, 6), blur=12, color=PANEL_SHADOW):
    shadow_surface = pygame.Surface((rect.width + blur * 2, rect.height + blur * 2), pygame.SRCALPHA)
    shadow_rect = pygame.Rect(blur, blur, rect.width, rect.height)
    pygame.draw.rect(shadow_surface, color, shadow_rect, border_radius=16)
    screen.blit(shadow_surface, rect.move(offset[0] - blur, offset[1] - blur))

def draw_button(rect, label, top_color, bottom_color, enabled=True):
    draw_shadow(rect, offset=(3, 4), blur=10, color=BUTTON_SHADOW_COLOR)
    button_surface = pygame.Surface(rect.size, pygame.SRCALPHA)
    draw_vertical_gradient(button_surface, button_surface.get_rect(), top_color, bottom_color)
    pygame.draw.rect(button_surface, (255, 255, 255, 40), button_surface.get_rect(), border_radius=10)
    if not enabled:
        dim = pygame.Surface(rect.size, pygame.SRCALPHA)
        dim.fill((255, 255, 255, 120))
        button_surface.blit(dim, (0, 0))
    screen.blit(button_surface, rect.topleft)
    pygame.draw.rect(screen, BUTTON_BORDER_COLOR, rect, 2, border_radius=10)
    label_color = BUTTON_TEXT_COLOR if enabled else BUTTON_DISABLED_TEXT
    text_surface = font_big.render(label, True, label_color)
    screen.blit(
        text_surface,
        (rect.centerx - text_surface.get_width() // 2,
         rect.centery - text_surface.get_height() // 2)
    )

def draw_marker(pos, fill_color, ring_color):
    if not pos:
        return
    glow_radius = GRID_SIZE
    glow_surface = pygame.Surface((glow_radius * 4, glow_radius * 4), pygame.SRCALPHA)
    pygame.draw.circle(glow_surface, (*fill_color, 40), (glow_radius * 2, glow_radius * 2), glow_radius * 2)
    screen.blit(glow_surface, (pos[0] - glow_radius * 2, pos[1] - glow_radius * 2), special_flags=pygame.BLEND_RGBA_ADD)
    pygame.draw.circle(screen, ring_color, pos, GRID_SIZE // 2 + 4, width=3)
    pygame.draw.circle(screen, fill_color, pos, GRID_SIZE // 2 - 2)

def draw_path_tiles(path):
    for cell in path:
        r = pygame.Rect(cell[0] * GRID_SIZE, cell[1] * GRID_SIZE, GRID_SIZE, GRID_SIZE)
        pygame.draw.rect(screen, PATH_COLOR, r, border_radius=6)
        pygame.draw.rect(screen, PATH_BORDER_COLOR, r, 1, border_radius=6)

def draw_wall_preview_rect(rect):
    if not rect or rect.width <= 0 or rect.height <= 0:
        return
    preview_surface = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
    preview_surface.fill((120, 150, 210, 60))
    pygame.draw.rect(preview_surface, (120, 160, 255, 200), preview_surface.get_rect(), 2, border_radius=6)
    screen.blit(preview_surface, rect.topleft)

def draw_robot_cell(cell):
    if not cell:
        return
    pos = cell_center(cell)
    radius = GRID_SIZE // 3 + 2
    glow_surface = pygame.Surface((radius * 6, radius * 6), pygame.SRCALPHA)
    pygame.draw.circle(glow_surface, ROBOT_COLOR + (80,), (radius * 3, radius * 3), radius * 3)
    screen.blit(glow_surface, (pos[0] - radius * 3, pos[1] - radius * 3), special_flags=pygame.BLEND_RGBA_ADD)
    pygame.draw.circle(screen, ROBOT_RING_COLOR, pos, radius + 4, width=4)
    pygame.draw.circle(screen, ROBOT_COLOR, pos, radius)
    pygame.draw.circle(screen, ROBOT_CORE_COLOR, pos, radius // 2)

def clamp(n, lo, hi):
    return max(lo, min(hi, n))

def grid_cell_from_pixel(p):
    return p[0] // GRID_SIZE, p[1] // GRID_SIZE

def cell_center(cell):
    cx, cy = cell
    return (cx * GRID_SIZE + GRID_SIZE // 2,
            cy * GRID_SIZE + GRID_SIZE // 2)

def snap_pixel_to_cell_center(p):
    cell = grid_cell_from_pixel(p)
    max_x = SCREEN_W // GRID_SIZE - 1
    max_y = SCREEN_H // GRID_SIZE - 1
    cell = (clamp(cell[0], 0, max_x), clamp(cell[1], 0, max_y))
    return cell, cell_center(cell)

def draw_grid():
    # Only draw grid on the "world" area (not on the instruction panel)
    for x in range(PANEL_W, SCREEN_W, GRID_SIZE):
        color = GRID_ACCENT if (x // GRID_SIZE) % 5 == 0 else GRID_COLOR
        pygame.draw.line(screen, color, (x, 0), (x, SCREEN_H))
    for y in range(0, SCREEN_H, GRID_SIZE):
        color = GRID_ACCENT if (y // GRID_SIZE) % 5 == 0 else GRID_COLOR
        pygame.draw.line(screen, color, (PANEL_W, y), (SCREEN_W, y))

def draw_background():
    draw_vertical_gradient(screen, pygame.Rect(0, 0, SCREEN_W, SCREEN_H), BG_TOP_COLOR, BG_BOTTOM_COLOR)
    overlay = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
    pygame.draw.circle(overlay, (255, 255, 255, 35), (SCREEN_W - 120, 90), 220)
    pygame.draw.circle(overlay, (255, 255, 255, 45), (SCREEN_W - 320, -20), 260)
    pygame.draw.circle(overlay, (255, 255, 255, 30), (SCREEN_W - 60, 280), 160)
    pygame.draw.circle(overlay, (255, 168, 120, 25), (SCREEN_W - 180, 460), 260)
    screen.blit(overlay, (0, 0), special_flags=pygame.BLEND_RGBA_ADD)

class Wall:
    def __init__(self, rect: pygame.Rect):
        self.rect = rect

    def draw(self):
        pygame.draw.rect(screen, WALL_COLOR, self.rect, border_radius=4)
        top_slice = pygame.Rect(self.rect.x, self.rect.y, self.rect.width, max(2, self.rect.height // 4))
        pygame.draw.rect(screen, WALL_HILITE, top_slice, border_radius=4)
        pygame.draw.rect(screen, PATH_BORDER_COLOR, self.rect, 2, border_radius=4)

def rect_from_drag(a, b):
    x1, y1 = a
    x2, y2 = b
    rect = pygame.Rect(min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1))
    rect.normalize()
    return rect

def snap_rect_to_grid(rect):
    x1 = (rect.left  // GRID_SIZE) * GRID_SIZE
    y1 = (rect.top   // GRID_SIZE) * GRID_SIZE
    x2 = ((rect.right  + GRID_SIZE - 1) // GRID_SIZE) * GRID_SIZE
    y2 = ((rect.bottom + GRID_SIZE - 1) // GRID_SIZE) * GRID_SIZE
    snapped = pygame.Rect(x1, y1, max(0, x2 - x1), max(0, y2 - y1))
    snapped.normalize()
    return snapped

def astar(start_cell, goal_cell, walls):
    sx, sy = start_cell
    gx, gy = goal_cell

    max_x = SCREEN_W // GRID_SIZE
    max_y = SCREEN_H // GRID_SIZE

    def walkable(x, y):
        if not (0 <= x < max_x and 0 <= y < max_y):
            return False
        rect = pygame.Rect(x * GRID_SIZE, y * GRID_SIZE, GRID_SIZE, GRID_SIZE)
        for w in walls:
            if rect.colliderect(w.rect):
                return False
        return True

    def neighbors(node):
        x, y = node
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x + dx, y + dy
            if walkable(nx, ny):
                yield (nx, ny)

    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = [(h((sx, sy), (gx, gy)), 0, (sx, sy))]
    heapq.heapify(open_set)
    came_from = {}
    gscore = {(sx, sy): 0}
    seen = set()

    while open_set:
        _, cost, cur = heapq.heappop(open_set)
        if cur in seen:
            continue
        seen.add(cur)

        if cur == (gx, gy):
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return list(reversed(path))

        for nb in neighbors(cur):
            tentative = cost + 1
            if nb not in gscore or tentative < gscore[nb]:
                gscore[nb] = tentative
                f = tentative + h(nb, (gx, gy))
                heapq.heappush(open_set, (f, tentative, nb))
                came_from[nb] = cur

    return None

def path_to_instructions(path):
    """
    Convert a cell-by-cell path into compact instructions.
    Example: RIGHT x3, DOWN x2, ...
    """
    if not path or len(path) < 2:
        return []

    def step_dir(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if dx == 1 and dy == 0:  return "RIGHT"
        if dx == -1 and dy == 0: return "LEFT"
        if dx == 0 and dy == 1:  return "DOWN"
        if dx == 0 and dy == -1: return "UP"
        return "?"

    instructions = []
    cur_dir = step_dir(path[0], path[1])
    count = 1

    for i in range(1, len(path) - 1):
        d = step_dir(path[i], path[i + 1])
        if d == cur_dir:
            count += 1
        else:
            instructions.append(f"MOVE {cur_dir} x{count}")
            cur_dir = d
            count = 1

    instructions.append(f"MOVE {cur_dir} x{count}")
    return instructions

def draw_panel(instructions, status_text=None):
    panel_rect = pygame.Rect(0, 0, PANEL_W, SCREEN_H)
    draw_shadow(panel_rect, offset=(6, 8))
    draw_rounded_gradient(screen, panel_rect, PANEL_TOP_COLOR, PANEL_BOTTOM_COLOR, radius=20)
    pygame.draw.rect(screen, PANEL_BORDER, panel_rect, 2, border_radius=20)

    y = 20
    title = font_big.render("Robot Path Studio", True, TEXT_COLOR)
    screen.blit(title, (20, y))
    y += 34

    tips = [
        "Left click: Start → End",
        "Drag mouse: Walls",
        "Go / Space: Solve + run",
        "R / Reset: Clear grid"
    ]
    for tip in tips:
        tip_surf = font.render(tip, True, TEXT_MUTED)
        screen.blit(tip_surf, (20, y))
        y += 22

    y += 6
    pygame.draw.line(screen, PANEL_BORDER, (20, y), (PANEL_W - 20, y), width=2)
    y += 12

    if status_text:
        warn = "UNSOLVABLE" in status_text
        status_color = STATUS_WARN_BG if warn else STATUS_OK_BG
        status_surface = pygame.Surface((PANEL_W - 40, 36), pygame.SRCALPHA)
        status_surface.fill(status_color)
        screen.blit(status_surface, (20, y))
        st_color = (200, 40, 60) if warn else (30, 100, 160)
        st = font.render(status_text, True, st_color)
        screen.blit(st, (30, y + 8))
        y += 48

    if not instructions:
        msg = font.render("Press Go to chart a route ↗", True, TEXT_MUTED)
        screen.blit(msg, (20, y))
        return

    max_lines = (SCREEN_H - y - 20) // 22
    for line in instructions[:max_lines]:
        surf = font.render(line, True, TEXT_COLOR)
        screen.blit(surf, (20, y))
        y += 22

def reset():
    global start_cell, end_cell, start_pos, end_pos
    global walls, dragging_wall, wall_start_px, wall_preview
    global path_found, robot_path, robot_idx, moving, last_step_time
    global instructions, status_text

    start_cell = None
    end_cell   = None
    start_pos  = None
    end_pos    = None

    walls = []

    dragging_wall = False
    wall_start_px = None
    wall_preview  = None

    path_found = None
    robot_path = None
    robot_idx  = 0
    moving     = False
    last_step_time = 0

    instructions = []
    status_text = None

def compute_path():
    global path_found, robot_path, robot_idx, moving, last_step_time
    global instructions, status_text

    status_text = None
    instructions = []

    if start_cell and end_cell:
        path_found = astar(start_cell, end_cell, walls)
        if path_found:
            robot_path = path_found
            robot_idx = 0
            moving = True
            last_step_time = pygame.time.get_ticks()

            # Build instruction set
            instructions = []
            instructions.append(f"START at {start_cell}")
            instructions.extend(path_to_instructions(path_found))
            instructions.append(f"END at {end_cell}")
        else:
            status_text = "UNSOLVABLE"
            robot_path = None
            moving = False
    else:
        status_text = "Place START then END first."

reset()

# --------------------------------------------------------------------------- #
running = True
while running:
    clock.tick(FPS)
    now = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # ----------------------- keyboard -------------------------------------
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                reset()
            if event.key == pygame.K_SPACE and not moving:
                compute_path()

        # ----------------------- mouse ----------------------------------------
        if event.type == pygame.MOUSEBUTTONDOWN:
            # UI clicks first
            if btn_rect.collidepoint(event.pos):
                reset()
                continue
            if go_rect.collidepoint(event.pos):
                if not moving:
                    compute_path()
                continue

            # Ignore clicks inside the panel area (so you don't draw in it)
            if event.pos[0] < PANEL_W:
                continue

            # Left click: place start, then end; then walls
            if event.button == 1 and not dragging_wall:
                cell, pos = snap_pixel_to_cell_center(event.pos)
                if start_cell is None:
                    start_cell, start_pos = cell, pos
                elif end_cell is None:
                    end_cell, end_pos = cell, pos
                else:
                    dragging_wall = True
                    wall_start_px = event.pos
                    wall_preview = None

            # Right click: wall drag anytime
            if event.button == 3 and not dragging_wall:
                dragging_wall = True
                wall_start_px = event.pos
                wall_preview = None

        if event.type == pygame.MOUSEMOTION and dragging_wall:
            wall_preview = rect_from_drag(wall_start_px, event.pos)
            wall_preview = snap_rect_to_grid(wall_preview)

        if event.type == pygame.MOUSEBUTTONUP and dragging_wall:
            if wall_preview and wall_preview.width > 0 and wall_preview.height > 0:
                ok = True
                if start_pos and wall_preview.collidepoint(start_pos):
                    ok = False
                if end_pos and wall_preview.collidepoint(end_pos):
                    ok = False
                if ok:
                    walls.append(Wall(wall_preview.copy()))
            dragging_wall = False
            wall_start_px = None
            wall_preview  = None

    # --------------------------------------------------------------------- update robot
    if moving and robot_path:
        if now - last_step_time >= STEP_MS:
            last_step_time = now
            robot_idx += 1
            if robot_idx >= len(robot_path):
                moving = False
                robot_idx = len(robot_path) - 1

    # --------------------------------------------------------------------- draw
    draw_background()
    draw_panel(instructions, status_text)
    draw_grid()

    # Walls
    for w in walls:
        w.draw()

    # Wall preview
    if wall_preview:
        draw_wall_preview_rect(wall_preview)

    # Path visualization
    if path_found:
        draw_path_tiles(path_found)

    # Start / End markers
    draw_marker(start_pos, START_COLOR, START_RING_COLOR)
    draw_marker(end_pos, END_COLOR, END_RING_COLOR)

    # Robot
    if robot_path and robot_idx >= 0:
        draw_robot_cell(robot_path[robot_idx])

    draw_button(btn_rect, "Reset", RESET_BTN_TOP, RESET_BTN_BOTTOM, enabled=True)
    draw_button(go_rect, "Go", GO_BTN_TOP, GO_BTN_BOTTOM, enabled=not moving)

    pygame.display.flip()

pygame.quit()
sys.exit()
