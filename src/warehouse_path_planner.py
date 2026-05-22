"""
Robot Path Simulator  –  Enhanced Edition
==========================================
New features:
  • Algorithm selector: A*, Dijkstra, Greedy Best-First, BFS
  • Diagonal movement toggle (8-connected vs 4-connected)
  • Step-by-step exploration visualiser (explored / frontier overlays)
  • Terrain / cost-map painting (plain, mud, slope, road)
  • Multiple waypoints  – numbered checkpoints, piecewise A*
  • Smooth sub-cell robot interpolation
  • Speed slider (keys + / -)
  • Erase walls: middle-click or E + left-click
  • Save / Load map  (JSON)
  • Procedural maze generator  (recursive back-tracker)
  • Richer metrics: nodes explored, path cost, solve time, turn count
  • Export route instructions to .txt
"""

import heapq, json, math, os, random, sys, time
import pygame

# ─────────────────────────────────────────────────── layout constants ──── #
SCREEN_W  = 1200
SCREEN_H  = 760
FPS       = 60
GRID_SIZE = 20
PANEL_W   = 320
WORLD_X   = PANEL_W + 24
WORLD_Y   = 28
WORLD_W   = SCREEN_W - WORLD_X - 18
WORLD_H   = SCREEN_H - WORLD_Y - 18
WORLD_COLS = WORLD_W // GRID_SIZE
WORLD_ROWS = WORLD_H // GRID_SIZE

# ─────────────────────────────────────────────────────── colour theme ───── #
# Industrial light palette: concrete whites, steel greys, safety yellow/orange
# No dark backgrounds — all panels stay light; accents are hazard-coded.

START_COLOR       = (24,  148,  68)   # safety green
START_RING_COLOR  = (14,   98,  44)
END_COLOR         = (210,  60,  40)   # hazard red
END_RING_COLOR    = (148,  36,  22)
WAYPOINT_COLORS   = [(224,168, 20),(194, 88, 30),(40,138,180),(130, 90,170)]
WALL_COLOR        = (108, 112, 116)   # steel grey — shelving unit
WALL_HILITE       = (158, 164, 170)   # brushed-metal highlight
PATH_COLOR        = (255, 220,  60)   # safety yellow — painted floor lane
PATH_BORDER_COLOR = (186, 148,  10)   # darker yellow border
EXPLORED_COLOR    = (200, 220, 195, 100)
FRONTIER_COLOR    = (255, 210, 100, 120)
ROBOT_COLOR       = (218,  82,  20)   # safety orange AMR body
ROBOT_CORE_COLOR  = (255, 255, 255)
ROBOT_RING_COLOR  = (140,  46,   8)
TEXT_COLOR        = (28,   30,  32)   # near-black, not blue
TEXT_MUTED        = (90,   94, 100)
TEXT_DIM          = (148, 152, 158)
ACCENT_2          = (186, 148,  10)   # yellow-gold accent (matches path)
GRID_COLOR        = (214, 212, 208)   # warm concrete grid
GRID_ACCENT       = (192, 188, 182)   # stronger grid every 5

# Backgrounds — warm off-white concrete, not blue-white
BG_TOP_COLOR       = (245, 243, 238)
BG_BOTTOM_COLOR    = (228, 224, 216)
PANEL_TOP_COLOR    = (252, 250, 246)   # light cream panel
PANEL_BOTTOM_COLOR = (238, 234, 226)
PANEL_BORDER       = (186, 180, 170)   # warm grey border
PANEL_SHADOW       = (60,  50,  30,  60)

# Buttons — industrial-coded colours
BUTTON_TEXT_COLOR    = (255, 255, 255)
BUTTON_DISABLED_TEXT = (196, 192, 184)
BUTTON_BORDER_COLOR  = (255, 255, 255)
RESET_BTN_TOP        = (128, 122, 114)   # steel grey reset
RESET_BTN_BOTTOM     = (96,   92,  84)
GO_BTN_TOP           = (38,  148,  72)   # safety green go
GO_BTN_BOTTOM        = (22,  110,  52)
SAVE_BTN_TOP         = (60,  130, 180)   # process blue save
SAVE_BTN_BOTTOM      = (36,   96, 140)
LOAD_BTN_TOP         = (80,  148, 200)
LOAD_BTN_BOTTOM      = (48,  108, 158)
MAZE_BTN_TOP         = (210, 130,  30)   # amber generate
MAZE_BTN_BOTTOM      = (170,  94,  14)
BUTTON_SHADOW_COLOR  = (40,  36,  24, 100)

STATUS_WARN_BG = (255, 220, 200, 180)   # warm orange-red warning
STATUS_OK_BG   = (220, 245, 210, 180)   # safety green ok

# Terrain — warehouse zone colours (all light; no dark tones)
TERRAINS = {
    "plain": ((250, 248, 244), None,          1.0),   # bare concrete
    "road":  ((218, 240, 210), (190, 220, 182), 0.5), # marked fast lane (green stripe)
    "mud":   ((230, 210, 170), (210, 188, 148), 3.0), # congested zone (amber)
    "slope": ((240, 222, 196), (218, 198, 168), 2.0), # incline / ramp (warm)
}
TERRAIN_ORDER = ["plain", "road", "mud", "slope"]
TERRAIN_LABELS = {
    "plain": "Concrete",
    "road":  "Fast Lane",
    "mud":   "Congested",
    "slope": "Ramp",
}
TERRAIN_KEY_COLORS = {
    "plain": (224, 220, 212),
    "road":  (180, 228, 170),
    "mud":   (228, 200, 148),
    "slope": (228, 204, 170),
}

ALGORITHMS = ["A*", "Dijkstra", "Greedy", "BFS"]

# ──────────────────────────────────────────────────────── pygame init ────── #
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("AMR Warehouse Path Planner")
clock      = pygame.time.Clock()
font       = pygame.font.SysFont(None, 22)
font_big   = pygame.font.SysFont(None, 27)
font_title = pygame.font.SysFont(None, 34, bold=True)
font_label = pygame.font.SysFont(None, 19, bold=True)
font_tiny  = pygame.font.SysFont(None, 16)

world_rect = pygame.Rect(WORLD_X, WORLD_Y,
                         WORLD_COLS * GRID_SIZE, WORLD_ROWS * GRID_SIZE)

# ──────────────────────────────────── drawing helpers ─────────────────── #
def lerp_color(c1, c2, t):
    n = len(c1)
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(n))

def draw_vertical_gradient(surface, rect, top, bottom):
    h = rect.height
    if h <= 0:
        return
    for y in range(h):
        c = lerp_color(top, bottom, y / max(1, h - 1))
        pygame.draw.line(surface, c,
                         (rect.left, rect.top + y),
                         (rect.right - 1, rect.top + y))

def draw_rounded_gradient(surface, rect, top, bottom, radius=12):
    gs = pygame.Surface(rect.size, pygame.SRCALPHA)
    draw_vertical_gradient(gs, gs.get_rect(), top, bottom)
    mask = pygame.Surface(rect.size, pygame.SRCALPHA)
    pygame.draw.rect(mask, (255, 255, 255), mask.get_rect(), border_radius=radius)
    gs.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    surface.blit(gs, rect.topleft)

def draw_shadow(rect, offset=(6, 6), blur=12, color=PANEL_SHADOW):
    ss = pygame.Surface((rect.width + blur * 2, rect.height + blur * 2),
                        pygame.SRCALPHA)
    sr = pygame.Rect(blur, blur, rect.width, rect.height)
    pygame.draw.rect(ss, color, sr, border_radius=16)
    screen.blit(ss, rect.move(offset[0] - blur, offset[1] - blur))

def draw_text(text, pos, color=TEXT_COLOR, use_font=None, anchor="topleft"):
    surf = (use_font or font).render(text, True, color)
    r = surf.get_rect()
    setattr(r, anchor, pos)
    screen.blit(surf, r)
    return r

def draw_button(rect, label, top_color, bottom_color,
                enabled=True, hovered=False, small=False):
    # Outer drop shadow — warm grey, not blue
    draw_shadow(rect, offset=(2, 3), blur=6, color=BUTTON_SHADOW_COLOR)
    bs = pygame.Surface(rect.size, pygame.SRCALPHA)
    tc, bc = top_color, bottom_color
    if hovered and enabled:
        tc = lerp_color(tc, (255, 255, 255), 0.15)
        bc = lerp_color(bc, (255, 255, 255), 0.15)
    # Flat gradient — no rounded corners, industrial look
    draw_vertical_gradient(bs, bs.get_rect(), tc, bc)
    # Subtle inner highlight on top edge (machined surface)
    pygame.draw.line(bs, (255,255,255,70), (2,1), (rect.width-3,1), 1)
    if not enabled:
        dim = pygame.Surface(rect.size, pygame.SRCALPHA)
        dim.fill((255, 255, 255, 100))
        bs.blit(dim, (0,0))
    # Sharp-cornered rect (border_radius=4 only — looks bolted, not bubbly)
    mask = pygame.Surface(rect.size, pygame.SRCALPHA)
    pygame.draw.rect(mask, (255,255,255), mask.get_rect(), border_radius=4)
    bs.blit(mask, (0,0), special_flags=pygame.BLEND_RGBA_MULT)
    screen.blit(bs, rect.topleft)
    # 2-px border, slightly dark — like a stamped metal edge
    pygame.draw.rect(screen, (255,255,255,180), rect, 1, border_radius=4)
    pygame.draw.rect(screen, lerp_color(bc,(0,0,0),0.3), rect, 2, border_radius=4)
    lc = BUTTON_TEXT_COLOR if enabled else BUTTON_DISABLED_TEXT
    f  = font if small else font_big
    ts = f.render(label, True, lc)
    screen.blit(ts, (rect.centerx - ts.get_width()//2,
                     rect.centery - ts.get_height()//2))

def draw_metric_card(rect, label, value, color):
    # Cream card base
    pygame.draw.rect(screen, (248, 244, 236), rect, border_radius=4)
    pygame.draw.rect(screen, PANEL_BORDER, rect, 1, border_radius=4)
    # Coloured top indicator bar — like an instrument gauge
    bar = pygame.Rect(rect.x, rect.y, rect.width, 4)
    pygame.draw.rect(screen, color, bar, border_radius=2)
    draw_text(label, (rect.x + 8, rect.y + 9),  TEXT_DIM,   font_label)
    draw_text(value, (rect.x + 8, rect.y + 26), color,       font_big)

def draw_choice_grid(state, items, active_value, rect_store, x, y, cols=2, cell_h=28):
    cell_w = (PANEL_W - 42) // cols
    for i, (label, value) in enumerate(items):
        r = pygame.Rect(x + (i % cols) * (cell_w + 6), y + (i // cols) * (cell_h + 6), cell_w, cell_h)
        active = active_value == value
        pygame.draw.rect(screen, ACCENT_2 if active else (248, 251, 255), r, border_radius=8)
        pygame.draw.rect(screen, ACCENT_2 if active else PANEL_BORDER, r, 2 if active else 1, border_radius=8)
        draw_text(label, r.center, (255, 255, 255) if active else TEXT_COLOR, font_label, anchor="center")
        rect_store[value] = r
    return y + math.ceil(len(items) / cols) * (cell_h + 6)

# ────────────────────────────────── grid geometry helpers ─────────────── #
def clamp(n, lo, hi):
    return max(lo, min(hi, n))

def grid_cell_from_pixel(p):
    return (p[0] - WORLD_X) // GRID_SIZE, (p[1] - WORLD_Y) // GRID_SIZE

def cell_center(cell):
    cx, cy = cell
    return (WORLD_X + cx * GRID_SIZE + GRID_SIZE // 2,
            WORLD_Y + cy * GRID_SIZE + GRID_SIZE // 2)

def cell_rect(cell):
    return pygame.Rect(WORLD_X + cell[0] * GRID_SIZE,
                       WORLD_Y + cell[1] * GRID_SIZE,
                       GRID_SIZE, GRID_SIZE)

def snap_pixel_to_cell(p):
    cx = clamp((p[0] - WORLD_X) // GRID_SIZE, 0, WORLD_COLS - 1)
    cy = clamp((p[1] - WORLD_Y) // GRID_SIZE, 0, WORLD_ROWS - 1)
    cell = (cx, cy)
    return cell, cell_center(cell)

def snap_rect_to_grid(rect):
    rect = rect.clip(world_rect)
    x1 = WORLD_X + ((rect.left  - WORLD_X) // GRID_SIZE) * GRID_SIZE
    y1 = WORLD_Y + ((rect.top   - WORLD_Y) // GRID_SIZE) * GRID_SIZE
    x2 = WORLD_X + (((rect.right  - WORLD_X) + GRID_SIZE - 1) // GRID_SIZE) * GRID_SIZE
    y2 = WORLD_Y + (((rect.bottom - WORLD_Y) + GRID_SIZE - 1) // GRID_SIZE) * GRID_SIZE
    x1 = clamp(x1, world_rect.left, world_rect.right)
    x2 = clamp(x2, world_rect.left, world_rect.right)
    y1 = clamp(y1, world_rect.top,  world_rect.bottom)
    y2 = clamp(y2, world_rect.top,  world_rect.bottom)
    return pygame.Rect(x1, y1, max(0, x2 - x1), max(0, y2 - y1))

def rect_from_drag(a, b):
    x1, y1 = a; x2, y2 = b
    r = pygame.Rect(min(x1,x2), min(y1,y2), abs(x2-x1), abs(y2-y1))
    r.normalize()
    return r

def in_world(pos):
    return world_rect.collidepoint(pos)

# ───────────────────────────────────────── Wall + terrain map ─────────── #
class Wall:
    def __init__(self, rect: pygame.Rect):
        self.rect = rect

    def draw(self):
        pygame.draw.rect(screen, WALL_COLOR, self.rect, border_radius=4)
        top = pygame.Rect(self.rect.x, self.rect.y,
                          self.rect.width, max(2, self.rect.height // 4))
        pygame.draw.rect(screen, WALL_HILITE, top, border_radius=4)
        pygame.draw.rect(screen, (58, 67, 80), self.rect, 2, border_radius=4)
        if self.rect.width >= GRID_SIZE * 2 and self.rect.height >= GRID_SIZE:
            for x in range(self.rect.left + GRID_SIZE, self.rect.right, GRID_SIZE * 2):
                pygame.draw.line(screen, (112, 124, 140), (x, self.rect.top + 3), (x, self.rect.bottom - 3), 1)

    def as_dict(self):
        return {"x": self.rect.x, "y": self.rect.y,
                "w": self.rect.width, "h": self.rect.height}

    @staticmethod
    def from_dict(d):
        return Wall(pygame.Rect(d["x"], d["y"], d["w"], d["h"]))

# terrain_map: {(col,row): terrain_name}
terrain_map: dict = {}

def terrain_cost(cell):
    t = terrain_map.get(cell, "plain")
    return TERRAINS[t][2]

def is_wall_cell(cell, walls):
    r = cell_rect(cell)
    return any(r.colliderect(w.rect) for w in walls)

def walkable(x, y, walls, diagonal=False):
    if not (0 <= x < WORLD_COLS and 0 <= y < WORLD_ROWS):
        return False
    return not is_wall_cell((x, y), walls)

# ──────────────────────────────────────────── path-finding algorithms ─── #
def get_neighbors(node, walls, diagonal):
    x, y = node
    dirs4 = [(1,0),(-1,0),(0,1),(0,-1)]
    dirs8 = dirs4 + [(1,1),(1,-1),(-1,1),(-1,-1)]
    for dx, dy in (dirs8 if diagonal else dirs4):
        nx, ny = x + dx, y + dy
        if not walkable(nx, ny, walls):
            continue
        # diagonal: both cardinal neighbours must be free (no corner-cutting)
        if diagonal and dx != 0 and dy != 0:
            if not walkable(x + dx, y, walls) or not walkable(x, y + dy, walls):
                continue
        cost = math.hypot(dx, dy) * terrain_cost((nx, ny))
        yield (nx, ny), cost

def heuristic(a, b, diagonal):
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    if diagonal:
        return math.sqrt(2) * min(dx, dy) + abs(dx - dy)   # Chebyshev
    return dx + dy                                          # Manhattan

def run_algorithm(algo, start, goal, walls, diagonal):
    """
    Returns (path, explored_order, frontier_at_end, nodes_explored, total_cost)
    path is None if no solution.
    explored_order is list of cells in the order they were closed.
    """
    open_set   = []
    came_from  = {}
    gscore     = {start: 0.0}
    seen       = set()
    explored   = []     # closed nodes in order
    counter    = 0      # tie-breaker

    def push(cell, f):
        nonlocal counter
        heapq.heappush(open_set, (f, counter, cell))
        counter += 1

    g0 = 0.0
    h0 = heuristic(start, goal, diagonal)

    if algo == "A*":
        push(start, g0 + h0)
    elif algo == "Dijkstra":
        push(start, g0)
    elif algo == "Greedy":
        push(start, h0)
    elif algo == "BFS":
        push(start, 0)

    while open_set:
        f, _, cur = heapq.heappop(open_set)
        if cur in seen:
            continue
        seen.add(cur)
        explored.append(cur)

        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            frontier = [item[2] for item in open_set if item[2] not in seen]
            return path, explored, frontier, len(seen), gscore.get(goal, 0)

        for nb, step_cost in get_neighbors(cur, walls, diagonal):
            if algo == "BFS":
                tentative = gscore[cur] + 1
            else:
                tentative = gscore[cur] + step_cost

            if nb not in gscore or tentative < gscore[nb]:
                gscore[nb] = tentative
                came_from[nb] = cur
                g  = tentative
                h  = heuristic(nb, goal, diagonal)
                if algo == "A*":
                    f = g + h
                elif algo == "Dijkstra":
                    f = g
                elif algo == "Greedy":
                    f = h
                else:  # BFS
                    f = g
                push(nb, f)

    frontier = [item[2] for item in open_set if item[2] not in seen]
    return None, explored, frontier, len(seen), float("inf")

# ─────────────────────────────────────── path utilities ───────────────── #
def path_to_instructions(path):
    if not path or len(path) < 2:
        return []

    def step_dir(a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        names = {(1,0):"E",(-1,0):"W",(0,1):"S",(0,-1):"N",
                 (1,1):"SE",(1,-1):"NE",(-1,1):"SW",(-1,-1):"NW"}
        return names.get((dx,dy),"?")

    instrs, cur_dir, count = [], step_dir(path[0], path[1]), 1
    for i in range(1, len(path)-1):
        d = step_dir(path[i], path[i+1])
        if d == cur_dir:
            count += 1
        else:
            instrs.append(f"MOVE {cur_dir} ×{count}")
            cur_dir, count = d, 1
    instrs.append(f"MOVE {cur_dir} ×{count}")
    return instrs

def count_turns(path):
    if len(path) < 3:
        return 0
    turns = 0
    def d(a,b): return (b[0]-a[0], b[1]-a[1])
    for i in range(1, len(path)-1):
        if d(path[i-1], path[i]) != d(path[i], path[i+1]):
            turns += 1
    return turns

# ───────────────────────────────── warehouse layout generator ─────────── #
def make_wall_cells(col, row, width, height):
    return pygame.Rect(
        WORLD_X + col * GRID_SIZE,
        WORLD_Y + row * GRID_SIZE,
        width * GRID_SIZE,
        height * GRID_SIZE,
    )

def generate_warehouse_layout():
    """Generate rack rows, cross aisles, staging blocks, and equipment zones."""
    new_walls = []

    # Long storage rack rows. Leave the left dock and several cross aisles open.
    dock_cols = 4
    rack_start = dock_cols + 2
    rack_end = WORLD_COLS - 4
    cross_aisles = {5, WORLD_ROWS // 2, WORLD_ROWS - 6}
    row = 3
    while row < WORLD_ROWS - 4:
        if row in cross_aisles or row + 1 in cross_aisles:
            row += 2
            continue

        x = rack_start
        while x < rack_end:
            bay_len = random.choice([5, 6, 7, 8])
            gap = random.choice([2, 3])
            if random.random() < 0.18:
                gap += 2
            width = min(bay_len, rack_end - x)
            if width >= 3:
                new_walls.append(Wall(make_wall_cells(x, row, width, 1)))
                if row + 1 < WORLD_ROWS - 4 and random.random() < 0.72:
                    new_walls.append(Wall(make_wall_cells(x, row + 1, width, 1)))
            x += bay_len + gap
        row += random.choice([4, 5])

    # Factory equipment / work cells. These are larger islands with open space around them.
    equipment_count = random.randint(4, 7)
    for _ in range(equipment_count):
        width = random.randint(3, 6)
        height = random.randint(2, 4)
        col = random.randint(rack_start, max(rack_start, rack_end - width))
        row = random.randint(4, max(4, WORLD_ROWS - height - 5))
        if row in cross_aisles or row + height in cross_aisles:
            continue
        new_walls.append(Wall(make_wall_cells(col, row, width, height)))

    # Shipping/staging pallets near dock edge, not fully blocking the dock lane.
    for row in range(8, WORLD_ROWS - 6, 8):
        if random.random() < 0.75:
            new_walls.append(Wall(make_wall_cells(1, row, 2, 2)))

    return new_walls

# ─────────────────────────────── drawing: world ───────────────────────── #
def draw_background():
    # Warm concrete base gradient
    draw_vertical_gradient(screen,
                           pygame.Rect(0, 0, SCREEN_W, SCREEN_H),
                           BG_TOP_COLOR, BG_BOTTOM_COLOR)
    # Subtle concrete texture: faint diagonal hatch lines
    hatch = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
    for i in range(0, SCREEN_W + SCREEN_H, 48):
        pygame.draw.line(hatch, (160,150,130, 14),
                         (i, 0), (0, i), 1)
    screen.blit(hatch, (0,0))

def draw_grid():
    # Floor shadow
    draw_shadow(world_rect, offset=(3,4), blur=10, color=(60,50,30,30))
    # Epoxy floor base — warm cream, not blue-white
    pygame.draw.rect(screen, (248, 245, 238), world_rect, border_radius=6)
    pygame.draw.rect(screen, PANEL_BORDER, world_rect, 2, border_radius=6)

    # Loading dock zone — hatched safety yellow/grey stripe pattern
    dock_w = GRID_SIZE * 3
    dock = pygame.Rect(world_rect.left, world_rect.top, dock_w, world_rect.height)
    pygame.draw.rect(screen, (244, 238, 224), dock)
    # diagonal hazard stripes on dock
    stripe_surf = pygame.Surface((dock.width, dock.height), pygame.SRCALPHA)
    stripe_col = (186,148,10, 28)   # faint safety yellow
    for i in range(-dock.height, dock.width + dock.height, 14):
        pygame.draw.line(stripe_surf, stripe_col,
                         (i, 0), (i + dock.height, dock.height), 5)
    screen.blit(stripe_surf, dock.topleft)
    # dock right-edge divider — bold yellow line like painted floor tape
    pygame.draw.line(screen, (210,168,14),
                     (dock.right, dock.top), (dock.right, dock.bottom), 3)

    # Grid lines — warm grey
    for col in range(WORLD_COLS + 1):
        x = WORLD_X + col * GRID_SIZE
        c = GRID_ACCENT if col % 5 == 0 else GRID_COLOR
        pygame.draw.line(screen, c, (x, WORLD_Y), (x, world_rect.bottom))
    for row in range(WORLD_ROWS + 1):
        y = WORLD_Y + row * GRID_SIZE
        c = GRID_ACCENT if row % 5 == 0 else GRID_COLOR
        pygame.draw.line(screen, c, (WORLD_X, y), (world_rect.right, y))

    # Dock label — stencil style, uppercase
    draw_text("LOADING DOCK", (dock.centerx, dock.bottom - 18),
              (160,140,100), font_tiny, anchor="center")

def draw_terrain():
    for (col, row), tname in terrain_map.items():
        _, fill, _ = TERRAINS[tname]
        if fill is None:
            continue
        r = cell_rect((col, row))
        pygame.draw.rect(screen, fill, r)

def draw_exploration(explored_cells, frontier_cells, show_exploration):
    if not show_exploration:
        return
    exp_surf  = pygame.Surface(world_rect.size, pygame.SRCALPHA)
    fron_surf = pygame.Surface(world_rect.size, pygame.SRCALPHA)
    for cell in explored_cells:
        r = pygame.Rect(cell[0]*GRID_SIZE, cell[1]*GRID_SIZE, GRID_SIZE, GRID_SIZE)
        pygame.draw.rect(exp_surf, EXPLORED_COLOR, r, border_radius=3)
    for cell in frontier_cells:
        r = pygame.Rect(cell[0]*GRID_SIZE, cell[1]*GRID_SIZE, GRID_SIZE, GRID_SIZE)
        pygame.draw.rect(fron_surf, FRONTIER_COLOR, r, border_radius=3)
    screen.blit(exp_surf,  world_rect.topleft)
    screen.blit(fron_surf, world_rect.topleft)

def draw_path_tiles(path, color=PATH_COLOR, border=PATH_BORDER_COLOR):
    for cell in path:
        r = cell_rect(cell)
        # Slightly inset so the path reads as painted floor tape
        inner = r.inflate(-3, -3)
        pygame.draw.rect(screen, color, inner, border_radius=3)
        pygame.draw.rect(screen, border, inner, 1, border_radius=3)

def draw_marker(pos, fill_color, ring_color, label=None):
    if not pos:
        return
    gr = GRID_SIZE
    gs = pygame.Surface((gr*4, gr*4), pygame.SRCALPHA)
    pygame.draw.circle(gs, (*fill_color,40), (gr*2,gr*2), gr*2)
    screen.blit(gs, (pos[0]-gr*2, pos[1]-gr*2), special_flags=pygame.BLEND_RGBA_ADD)
    pygame.draw.circle(screen, ring_color, pos, gr//2+4, width=3)
    pygame.draw.circle(screen, fill_color,  pos, gr//2-2)
    if label:
        draw_text(label, pos, (255,255,255), font_tiny, anchor="center")

def draw_robot(px, py):
    pos = (int(px), int(py))
    # AMR body — rectangular orange unit, not a soft blue circle
    hw = GRID_SIZE // 2 - 1
    body = pygame.Rect(pos[0]-hw, pos[1]-hw, hw*2, hw*2)
    pygame.draw.rect(screen, ROBOT_RING_COLOR, body.inflate(4,4), border_radius=3)
    pygame.draw.rect(screen, ROBOT_COLOR, body, border_radius=3)
    # White cross / beacon on top
    cx, cy = pos
    pygame.draw.line(screen, ROBOT_CORE_COLOR, (cx-4, cy), (cx+4, cy), 2)
    pygame.draw.line(screen, ROBOT_CORE_COLOR, (cx, cy-4), (cx, cy+4), 2)
    # Glow aura
    gs = pygame.Surface((hw*8, hw*8), pygame.SRCALPHA)
    pygame.draw.rect(gs, ROBOT_COLOR+(50,),
                     pygame.Rect(hw*2, hw*2, hw*4, hw*4), border_radius=6)
    screen.blit(gs, (pos[0]-hw*4, pos[1]-hw*4), special_flags=pygame.BLEND_RGBA_ADD)

def draw_wall_preview(rect):
    if not rect or rect.width <= 0 or rect.height <= 0:
        return
    ps = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
    ps.fill((108, 112, 116, 55))    # steel grey fill
    # Hazard-stripe border — orange dashed
    pygame.draw.rect(ps, (218, 110, 30, 200), ps.get_rect(), 2, border_radius=2)
    screen.blit(ps, rect.topleft)

def draw_world_header(mode, algo, diagonal):
    draw_text("WAREHOUSE FLOOR PLAN", (WORLD_X, 5), TEXT_COLOR, font_label)
    hints = {
        "place_wp":  "Click to place Dock / Stop / Drop-off points",
        "wall":      "Drag to draw storage racks or blocked aisles",
        "terrain":   "Paint fast lanes, congestion, or ramp zones",
        "erase":     "Drag to clear racks and blocked areas",
    }
    txt = f"{algo}  |  {'8-dir' if diagonal else 'Aisle'}  |  {hints.get(mode,'')}"
    draw_text(txt, (world_rect.right, 6), TEXT_MUTED, font_tiny, anchor="topright")

# ──────────────────────────────────── panel drawing ───────────────────── #
def draw_panel(state):
    state["algo_rects"] = [None] * len(ALGORITHMS)
    state["opt_rects"].clear()
    state["terrain_rects"] = [None] * len(TERRAIN_ORDER)
    state["mode_rects"].clear()

    # ── panel base — warm cream with ruled horizontal dividers ──────────
    panel_rect = pygame.Rect(0, 0, PANEL_W, SCREEN_H)
    draw_rounded_gradient(screen, panel_rect, PANEL_TOP_COLOR, PANEL_BOTTOM_COLOR, 0)

    # Right-edge — thick steel border stripe
    pygame.draw.rect(screen, (172, 164, 152), pygame.Rect(PANEL_W-4, 0, 4, SCREEN_H))
    pygame.draw.rect(screen, PANEL_BORDER, pygame.Rect(PANEL_W-1, 0, 1, SCREEN_H))

    # Header band — slightly darker cream, like a painted steel nameplate
    header_band = pygame.Rect(0, 0, PANEL_W-4, 64)
    pygame.draw.rect(screen, (234, 228, 216), header_band)
    pygame.draw.line(screen, PANEL_BORDER, (0, 64), (PANEL_W-4, 64), 2)
    # Yellow accent stripe at very top — caution tape header
    pygame.draw.rect(screen, (210, 168, 14), pygame.Rect(0, 0, PANEL_W-4, 5))

    y = 14
    draw_text("WAREHOUSE PLANNER", (18, y), TEXT_COLOR, font_title);  y += 28
    draw_text("AMR route planning  |  Rack & zone editor", (18, y), TEXT_MUTED, font); y += 34

    # ── progress stepper ─────────────────────────────────────────────────
    progress = ["Dock", "Stops", "Drop-off", "Route"]
    current  = min(len(state["waypoints"]), 3)
    step_w   = (PANEL_W - 36) // len(progress)
    for i, label in enumerate(progress):
        sx = 18 + i * step_w
        done   = i < current
        active = i == current
        # connector line
        if i < len(progress)-1:
            lc = (210,168,14) if done else PANEL_BORDER
            pygame.draw.line(screen, lc,
                             (sx + 14, y + 12), (sx + step_w - 2, y + 12), 2)
        # circle badge
        circ = pygame.Rect(sx, y, 24, 24)
        bg = (210,168,14) if done else ((34,148,68) if active else (224,220,212))
        pygame.draw.rect(screen, bg, circ, border_radius=12)
        pygame.draw.rect(screen, PANEL_BORDER, circ, 1, border_radius=12)
        tc = (255,255,255) if (done or active) else TEXT_DIM
        draw_text("✓" if done else str(i+1), circ.center, tc, font_tiny, anchor="center")
        draw_text(label, (sx+1, y+26), TEXT_COLOR if active else TEXT_DIM, font_tiny)
    y += 46

    # ── primary action buttons ────────────────────────────────────────────
    mp = pygame.mouse.get_pos()
    go_r    = pygame.Rect(18, y, PANEL_W-36, 40)
    reset_r = pygame.Rect(18,          y+47, (PANEL_W-48)//2, 32)
    maze_r  = pygame.Rect(18+(PANEL_W-48)//2+12, y+47, (PANEL_W-48)//2, 32)
    draw_button(go_r,    "▶  PLAN ROUTE", GO_BTN_TOP,   GO_BTN_BOTTOM,
                not state["moving"], go_r.collidepoint(mp))
    draw_button(reset_r, "CLEAR",         RESET_BTN_TOP, RESET_BTN_BOTTOM,
                hovered=reset_r.collidepoint(mp), small=True)
    draw_button(maze_r,  "GEN FLOOR",     MAZE_BTN_TOP,  MAZE_BTN_BOTTOM,
                hovered=maze_r.collidepoint(mp), small=True)
    state["go_rect"]    = go_r
    state["reset_rect"] = reset_r
    state["maze_rect"]  = maze_r
    y += 89

    # ── waypoint placement banner ─────────────────────────────────────────
    next_pt  = ("Dock" if len(state["waypoints"]) == 0
                else ("Drop-off" if len(state["waypoints"]) == 1
                      else "Pick Stop"))
    point_active = state["edit_mode"] == "place_wp"
    ph = pygame.Rect(18, y, PANEL_W-36, 28)
    # active = yellow tape highlight
    pygame.draw.rect(screen, (255,236,120) if point_active else (242,238,230),
                     ph, border_radius=3)
    pygame.draw.rect(screen, (186,148,10) if point_active else PANEL_BORDER,
                     ph, 2 if point_active else 1, border_radius=3)
    draw_text(f"▸  Place next: {next_pt}", ph.center,
              TEXT_COLOR, font_label, anchor="center")
    state["mode_rects"]["place_wp"] = ph
    y += 34
    draw_text("D=Dock   #=Stop   X=Drop-off", (20, y), TEXT_DIM, font_tiny)
    y += 16

    # ── section divider helper ────────────────────────────────────────────
    def ruled_section(label, yy):
        pygame.draw.line(screen, PANEL_BORDER, (18, yy), (PANEL_W-18, yy), 1)
        yy += 1
        # Section label on cream tag
        lw = font_label.size(label)[0] + 16
        pygame.draw.rect(screen, (236,230,218),
                         pygame.Rect(18, yy, lw, 18), border_radius=2)
        pygame.draw.rect(screen, PANEL_BORDER,
                         pygame.Rect(18, yy, lw, 18), 1, border_radius=2)
        draw_text(label, (26, yy+2), TEXT_DIM, font_label)
        return yy + 22

    y = ruled_section("FLOOR EDITING", y)
    modes = [("RACKS", "wall"), ("CLEAR", "erase"), ("ZONES", "terrain")]
    mw = (PANEL_W - 48) // 3
    for i, (lbl, mkey) in enumerate(modes):
        r = pygame.Rect(18+i*(mw+6), y, mw, 26)
        active = (state["edit_mode"] == mkey)
        # Active = amber/orange; inactive = steel plate
        bg = (210,130,20) if active else (234,230,222)
        pygame.draw.rect(screen, bg, r, border_radius=3)
        pygame.draw.rect(screen, (148, 90, 10) if active else PANEL_BORDER,
                         r, 2 if active else 1, border_radius=3)
        draw_text(lbl, r.center, (255,255,255) if active else TEXT_COLOR,
                  font_label, anchor="center")
        state["mode_rects"][mkey] = r
    y += 34

    y = ruled_section("ROUTING ALGORITHM", y)
    aw = (PANEL_W - 42) // 2
    for i, name in enumerate(ALGORITHMS):
        r = pygame.Rect(18+(i%2)*(aw+6), y+(i//2)*29, aw, 23)
        active = (state["algo"] == name)
        bg = (34,148,72) if active else (234,230,222)
        pygame.draw.rect(screen, bg, r, border_radius=3)
        pygame.draw.rect(screen, (14,98,44) if active else PANEL_BORDER,
                         r, 2 if active else 1, border_radius=3)
        draw_text(name, r.center, (255,255,255) if active else TEXT_MUTED,
                  font_label, anchor="center")
        state["algo_rects"][i] = r
    y += 65

    if state["edit_mode"] == "terrain" or state["advanced"]:
        y = ruled_section("WAREHOUSE ZONES", y)
        tw = (PANEL_W - 36) // len(TERRAIN_ORDER)
        for i, tname in enumerate(TERRAIN_ORDER):
            r = pygame.Rect(18+i*tw, y, tw-4, 22)
            active = (state["terrain_brush"] == tname)
            pygame.draw.rect(screen, TERRAIN_KEY_COLORS[tname], r, border_radius=3)
            pygame.draw.rect(screen, (60,50,30) if active else PANEL_BORDER,
                             r, 2 if active else 1, border_radius=3)
            draw_text(TERRAIN_LABELS[tname], r.center, TEXT_COLOR, font_tiny, anchor="center")
            state["terrain_rects"][i] = r
        y += 30

    y = ruled_section("PLANNER SETTINGS", y)
    adv_r = pygame.Rect(18, y, PANEL_W-36, 24)
    pygame.draw.rect(screen, (234,230,222), adv_r, border_radius=3)
    pygame.draw.rect(screen, PANEL_BORDER, adv_r, 1, border_radius=3)
    draw_text(("▾  Hide settings" if state["advanced"] else "▸  Show settings"),
              adv_r.center, TEXT_COLOR, font_label, anchor="center")
    state["advanced_rect"] = adv_r
    y += 32

    state["speed_rect"] = None
    if state["advanced"]:
        for label, key in [("Allow diagonal aisle cuts", "diagonal"),
                            ("Show search coverage",      "show_exploration")]:
            r = pygame.Rect(18, y, PANEL_W-36, 22)
            val = state[key]
            # Active = green tick row
            pygame.draw.rect(screen, (220,242,216) if val else (234,230,222),
                             r, border_radius=3)
            pygame.draw.rect(screen, PANEL_BORDER, r, 1, border_radius=3)
            mark = "✓" if val else "○"
            draw_text(f" {mark}  {label}", (r.x+6, r.y+4), TEXT_COLOR, font_label)
            state["opt_rects"][label] = r
            y += 26

        spd = state["speed"]
        draw_text(f"AMR speed  {spd}×", (18, y+1), TEXT_DIM, font_label)
        sr = pygame.Rect(108, y+6, PANEL_W-126, 9)
        pygame.draw.rect(screen, (218,212,200), sr, border_radius=4)
        fill_w = int(sr.width * (spd-1) / 9)
        pygame.draw.rect(screen, (210,130,20),
                         pygame.Rect(sr.x, sr.y, fill_w, sr.height), border_radius=4)
        pygame.draw.rect(screen, PANEL_BORDER, sr, 1, border_radius=4)
        state["speed_rect"] = sr
        y += 26

        bw = (PANEL_W-48)//2
        save_r   = pygame.Rect(18, y, bw, 28)
        load_r   = pygame.Rect(18+bw+12, y, bw, 28)
        export_r = pygame.Rect(18, y+34, PANEL_W-36, 26)
        draw_button(save_r,   "SAVE FLOOR",      SAVE_BTN_TOP, SAVE_BTN_BOTTOM,
                    hovered=save_r.collidepoint(mp), small=True)
        draw_button(load_r,   "LOAD FLOOR",      LOAD_BTN_TOP, LOAD_BTN_BOTTOM,
                    hovered=load_r.collidepoint(mp), small=True)
        draw_button(export_r, "EXPORT PICK ROUTE", RESET_BTN_TOP, RESET_BTN_BOTTOM,
                    bool(state["instructions"]), export_r.collidepoint(mp), small=True)
        state["save_rect"]   = save_r
        state["load_rect"]   = load_r
        state["export_rect"] = export_r
        y += 70
    else:
        state["save_rect"] = state["load_rect"] = state["export_rect"] = None

    # ── status banner ─────────────────────────────────────────────────────
    st = state["status_text"]
    if st:
        warn = "NO ROUTE" in st or "Error" in st
        sbg  = STATUS_WARN_BG if warn else STATUS_OK_BG
        ss   = pygame.Surface((PANEL_W-36, 32), pygame.SRCALPHA)
        ss.fill(sbg)
        # left colour bar
        pygame.draw.rect(ss, (200,60,40) if warn else (34,148,68),
                         pygame.Rect(0, 0, 4, 32))
        screen.blit(ss, (18, y))
        sc = (180,36,20) if warn else (14,100,48)
        draw_text(st, (28, y+9), sc, font_label)
        y += 40

    # ── metrics ───────────────────────────────────────────────────────────
    y = ruled_section("ROUTE METRICS", y)
    pw = (PANEL_W-48)//2
    metrics = [
        ("DIST",    str(len(state["path_found"])) if state["path_found"] else "-",
                    PATH_BORDER_COLOR),
        ("COST",    f"{state['path_cost']:.1f}" if state["path_cost"] else "—",
                    (186,120,20)),
        ("SEARCH",  str(state["nodes_explored"]),
                    (60,148,60)),
        ("AMR",     "RUN" if state["moving"] else ("READY" if state["robot_path"] else "IDLE"),
                    ROBOT_COLOR),
    ]
    if state["advanced"]:
        metrics.extend([
            ("TURNS", str(state["turns"]) if state["path_found"] else "—", (140,60,160)),
            ("MS",    f"{state['solve_ms']:.1f}" if state["solve_ms"] else "—",  TEXT_MUTED),
        ])
    for i, (lbl, val, col) in enumerate(metrics):
        r = pygame.Rect(18+(i%2)*(pw+12), y+(i//2)*58, pw, 50)
        draw_metric_card(r, lbl, val, col)
    metric_rows = math.ceil(len(metrics)/2)
    y += metric_rows*58 + 6

    # ── pick route instructions ───────────────────────────────────────────
    y = ruled_section("PICK ROUTE", y)
    instrs = state["instructions"]
    if not instrs:
        draw_text("Place Dock and Drop-off,", (18, y), TEXT_MUTED, font); y += 18
        draw_text("then press PLAN ROUTE.",   (18, y), TEXT_DIM,   font)
    else:
        max_lines = max(1, (SCREEN_H - y - 10) // 19)
        for line in instrs[:max_lines]:
            draw_text(line, (18, y), TEXT_COLOR, font); y += 19
        if len(instrs) > max_lines:
            draw_text(f"+ {len(instrs)-max_lines} more steps",
                      (18, y), TEXT_DIM, font_tiny)

# ──────────────────────────────── state management ────────────────────── #
def make_state():
    return {
        # map
        "walls":          [],
        "waypoints":      [],     # list of (cell, pos) — all points incl. start/end
        "terrain_map":    {},
        # path results
        "path_found":     None,
        "all_paths":      [],     # one per leg when multi-waypoint
        "robot_path":     None,
        "explored_cells": [],
        "frontier_cells": [],
        "nodes_explored": 0,
        "path_cost":      None,
        "turns":          0,
        "solve_ms":       None,
        # robot animation
        "moving":         False,
        "robot_t":        0.0,    # 0..1 between path cells
        "robot_idx":      0,      # current segment index
        "robot_px":       0.0,
        "robot_py":       0.0,
        "last_frame_ms":  0,
        # UI
        "algo":           "A*",
        "diagonal":       False,
        "show_exploration": True,
        "edit_mode":      "place_wp",   # wall | erase | terrain | place_wp
        "terrain_brush":  "mud",
        "speed":          3,        # 1–10
        "status_text":    None,
        "instructions":   [],
        # drag state
        "dragging_wall":  False,
        "wall_start_px":  None,
        "wall_preview":   None,
        "erase_dragging": False,
        # panel rect handles (populated by draw_panel)
        "algo_rects":    [None]*4,
        "opt_rects":     {},
        "terrain_rects": [None]*4,
        "mode_rects":    {},
        "go_rect":       None,
        "reset_rect":    None,
        "maze_rect":     None,
        "save_rect":     None,
        "load_rect":     None,
        "export_rect":   None,
        "speed_rect":    None,
        "advanced_rect": None,
        "advanced":      False,
        # placement mode
        "place_mode":    "start",   # start | end | idle
    }

def full_reset(st):
    st.update(make_state())
    terrain_map.clear()

def soft_reset_path(st):
    """Clear only path/robot, keep walls + waypoints."""
    st["path_found"]     = None
    st["all_paths"]      = []
    st["robot_path"]     = None
    st["explored_cells"] = []
    st["frontier_cells"] = []
    st["nodes_explored"] = 0
    st["path_cost"]      = None
    st["turns"]          = 0
    st["solve_ms"]       = None
    st["moving"]         = False
    st["robot_t"]        = 0.0
    st["robot_idx"]      = 0
    st["instructions"]   = []
    st["status_text"]    = None

def compute_path(st):
    soft_reset_path(st)
    wps = st["waypoints"]
    if len(wps) < 2:
        st["status_text"] = "Place a dock and drop-off first."
        return

    points = [w[0] for w in wps]
    t0 = time.perf_counter()
    full_path     = []
    all_explored  = []
    all_frontier  = []
    total_nodes   = 0
    total_cost    = 0.0

    for i in range(len(points)-1):
        path, exp, fron, n, cost = run_algorithm(
            st["algo"], points[i], points[i+1],
            st["walls"], st["diagonal"])
        if path is None:
            st["status_text"] = "NO ROUTE"
            return
        if i == 0:
            full_path.extend(path)
        else:
            full_path.extend(path[1:])
        all_explored.extend(exp)
        all_frontier.extend(fron)
        total_nodes += n
        total_cost  += cost

    dt_ms = (time.perf_counter() - t0) * 1000

    st["path_found"]     = full_path
    st["explored_cells"] = all_explored
    st["frontier_cells"] = all_frontier
    st["nodes_explored"] = total_nodes
    st["path_cost"]      = total_cost
    st["turns"]          = count_turns(full_path)
    st["solve_ms"]       = dt_ms
    st["robot_path"]     = full_path
    st["robot_idx"]      = 0
    st["robot_t"]        = 0.0
    c0 = cell_center(full_path[0])
    st["robot_px"], st["robot_py"] = float(c0[0]), float(c0[1])
    st["moving"]         = True
    st["last_frame_ms"]  = pygame.time.get_ticks()

    instrs = []
    instrs.append(f"DOCK {points[0]}")
    instrs.extend(path_to_instructions(full_path))
    instrs.append(f"DROP {points[-1]}")
    st["instructions"] = instrs
    st["status_text"]  = (f"{st['algo']} route · {len(full_path)} cells · "
                          f"{total_cost:.1f} cost · {dt_ms:.1f}ms")

def update_robot(st, now_ms):
    if not st["moving"] or not st["robot_path"]:
        return
    rp  = st["robot_path"]
    idx = st["robot_idx"]
    if idx >= len(rp) - 1:
        st["moving"]  = False
        st["robot_idx"] = len(rp) - 1
        cx, cy = cell_center(rp[-1])
        st["robot_px"], st["robot_py"] = float(cx), float(cy)
        return

    dt_ms   = now_ms - st["last_frame_ms"]
    st["last_frame_ms"] = now_ms
    speed   = st["speed"]
    # cells per second = speed; dt in seconds
    advance = (dt_ms / 1000.0) * speed
    st["robot_t"] += advance

    while st["robot_t"] >= 1.0 and st["robot_idx"] < len(rp)-1:
        st["robot_t"]  -= 1.0
        st["robot_idx"] += 1
        idx = st["robot_idx"]

    if st["robot_idx"] >= len(rp)-1:
        cx, cy = cell_center(rp[-1])
        st["robot_px"], st["robot_py"] = float(cx), float(cy)
        st["moving"] = False
        return

    t  = min(1.0, st["robot_t"])
    c0 = cell_center(rp[st["robot_idx"]])
    c1 = cell_center(rp[st["robot_idx"]+1])
    st["robot_px"] = c0[0] + (c1[0]-c0[0]) * t
    st["robot_py"] = c0[1] + (c1[1]-c0[1]) * t

# ─────────────────────────────── save / load ──────────────────────────── #
def save_map(st):
    path = get_save_path("json")
    if not path:
        return
    data = {
        "walls":     [w.as_dict() for w in st["walls"]],
        "waypoints": [[list(cell), list(pos)] for cell,pos in st["waypoints"]],
        "terrain":   {f"{k[0]},{k[1]}": v for k,v in terrain_map.items()},
    }
    try:
        with open(path,"w") as f:
            json.dump(data, f, indent=2)
        st["status_text"] = f"Saved: {os.path.basename(path)}"
    except Exception as e:
        st["status_text"] = f"Error: {e}"

def load_map(st):
    path = get_open_path("json")
    if not path:
        return
    try:
        with open(path) as f:
            data = json.load(f)
        full_reset(st)
        st["walls"]     = [Wall.from_dict(d) for d in data.get("walls",[])]
        st["waypoints"] = [(tuple(c), tuple(p))
                           for c,p in data.get("waypoints",[])]
        for k,v in data.get("terrain",{}).items():
            col,row = k.split(",")
            terrain_map[(int(col),int(row))] = v
        st["terrain_map"] = dict(terrain_map)
        st["status_text"] = f"Loaded: {os.path.basename(path)}"
    except Exception as e:
        st["status_text"] = f"Error: {e}"

def export_route(st):
    if not st["instructions"]:
        return
    path = get_save_path("txt")
    if not path:
        return
    try:
        with open(path,"w") as f:
            f.write("\n".join(st["instructions"]))
        st["status_text"] = f"Exported: {os.path.basename(path)}"
    except Exception as e:
        st["status_text"] = f"Error: {e}"

def get_save_path(ext):
    """Minimal cross-platform save dialog via pygame + input (no tkinter dep)."""
    # Just use a fixed filename with timestamp to avoid tkinter dependency
    import datetime
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"maze_save_{ts}.{ext}"

def get_open_path(ext):
    """Look for the most recent save file of the given extension."""
    files = [f for f in os.listdir(".") if f.endswith(f".{ext}") and f.startswith("maze_save_")]
    if not files:
        return None
    return sorted(files)[-1]

# ─────────────────────────────── erase helpers ────────────────────────── #
def erase_wall_at(px, py, st):
    if not in_world((px,py)):
        return
    cell, _ = snap_pixel_to_cell((px,py))
    cr = cell_rect(cell)
    st["walls"] = [w for w in st["walls"] if not cr.colliderect(w.rect)]

def paint_terrain_at(px, py, st):
    if not in_world((px,py)):
        return
    cell, _ = snap_pixel_to_cell((px,py))
    brush = st["terrain_brush"]
    if brush == "plain":
        terrain_map.pop(cell, None)
    else:
        terrain_map[cell] = brush

# ─────────────────────────────────── main loop ────────────────────────── #
def main():
    st = make_state()

    running = True
    while running:
        now = pygame.time.get_ticks()
        clock.tick(FPS)

        mp = pygame.mouse.get_pos()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

            # ── keyboard ──────────────────────────────────────────────
            if event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_r:
                    full_reset(st)
                elif k == pygame.K_SPACE and not st["moving"]:
                    compute_path(st)
                elif k == pygame.K_EQUALS or k == pygame.K_PLUS:
                    st["speed"] = min(10, st["speed"]+1)
                elif k == pygame.K_MINUS:
                    st["speed"] = max(1,  st["speed"]-1)
                elif k == pygame.K_d:
                    st["diagonal"] = not st["diagonal"]
                elif k == pygame.K_e:
                    st["edit_mode"] = "erase" if st["edit_mode"] != "erase" else "wall"
                elif k == pygame.K_t:
                    st["edit_mode"] = "terrain"
                elif k == pygame.K_w:
                    st["edit_mode"] = "wall"
                elif k == pygame.K_RETURN:
                    # finish waypoint placement
                    if st["edit_mode"] == "place_wp":
                        st["edit_mode"] = "wall"
                elif k == pygame.K_s and (event.mod & pygame.KMOD_CTRL):
                    save_map(st)
                elif k == pygame.K_o and (event.mod & pygame.KMOD_CTRL):
                    load_map(st)

            # ── mouse button down ──────────────────────────────────────
            if event.type == pygame.MOUSEBUTTONDOWN:
                # ── panel buttons ──────────────────────────────────────
                if st["go_rect"] and st["go_rect"].collidepoint(mp):
                    if not st["moving"]:
                        compute_path(st)
                    continue
                if st["reset_rect"] and st["reset_rect"].collidepoint(mp):
                    full_reset(st)
                    continue
                if st["maze_rect"] and st["maze_rect"].collidepoint(mp):
                    full_reset(st)
                    st["walls"] = generate_warehouse_layout()
                    st["edit_mode"] = "place_wp"
                    st["status_text"] = "Warehouse floor generated. Place dock and drop-off."
                    continue
                if st["save_rect"] and st["save_rect"].collidepoint(mp):
                    save_map(st)
                    continue
                if st["load_rect"] and st["load_rect"].collidepoint(mp):
                    load_map(st)
                    continue
                if st["export_rect"] and st["export_rect"].collidepoint(mp):
                    export_route(st)
                    continue
                if st["advanced_rect"] and st["advanced_rect"].collidepoint(mp):
                    st["advanced"] = not st["advanced"]
                    continue

                # ── algo selector ──────────────────────────────────────
                for i, r in enumerate(st["algo_rects"]):
                    if r and r.collidepoint(mp):
                        st["algo"] = ALGORITHMS[i]
                        soft_reset_path(st)
                        break

                # ── option toggles ─────────────────────────────────────
                for label, r in st["opt_rects"].items():
                    if r and r.collidepoint(mp):
                        key = "diagonal" if label=="Diagonal" else "show_exploration"
                        st[key] = not st[key]
                        if key == "diagonal":
                            soft_reset_path(st)

                # ── terrain brush ──────────────────────────────────────
                for i, r in enumerate(st["terrain_rects"]):
                    if r and r.collidepoint(mp):
                        st["terrain_brush"] = TERRAIN_ORDER[i]
                        st["edit_mode"] = "terrain"

                # ── edit mode buttons ──────────────────────────────────
                for mkey, r in st["mode_rects"].items():
                    if r and r.collidepoint(mp):
                        st["edit_mode"] = mkey

                # ── speed bar click ────────────────────────────────────
                sr = st.get("speed_rect")
                if sr and sr.collidepoint(mp):
                    frac = (mp[0]-sr.x) / sr.width
                    st["speed"] = max(1, min(10, int(frac*10)+1))

                # ── canvas interactions ────────────────────────────────
                if not in_world(mp):
                    continue

                mode = st["edit_mode"]
                cell, pos = snap_pixel_to_cell(mp)

                # waypoint placement (left click)
                if event.button == 1 and mode not in ("wall","erase","terrain"):
                    # first two clicks → start / end; more → intermediate waypoints
                    wps = st["waypoints"]
                    # check click on existing waypoint to remove
                    for i, (wc, wp) in enumerate(wps):
                        if math.hypot(mp[0]-wp[0], mp[1]-wp[1]) < GRID_SIZE:
                            wps.pop(i)
                            soft_reset_path(st)
                            break
                    else:
                        if len(wps) >= 2:
                            wps.insert(len(wps) - 1, (cell, pos))
                        else:
                            wps.append((cell, pos))
                        soft_reset_path(st)
                    continue

                if event.button == 1 and mode == "terrain":
                    paint_terrain_at(mp[0], mp[1], st)
                    continue

                if event.button == 1 and mode == "erase":
                    erase_wall_at(mp[0], mp[1], st)
                    soft_reset_path(st)
                    continue

                if event.button == 2:   # middle click → erase
                    erase_wall_at(mp[0], mp[1], st)
                    soft_reset_path(st)
                    continue

                # left/right drag → draw wall
                if event.button in (1, 3) and mode == "wall":
                    st["dragging_wall"] = True
                    st["wall_start_px"] = mp
                    st["wall_preview"]  = None

                # right-drag → erase mode
                if event.button == 3 and mode == "erase":
                    st["erase_dragging"] = True

            # ── mouse motion ───────────────────────────────────────────
            if event.type == pygame.MOUSEMOTION:
                if st["dragging_wall"] and st["wall_start_px"]:
                    raw = rect_from_drag(st["wall_start_px"], mp)
                    st["wall_preview"] = snap_rect_to_grid(raw)
                if st["erase_dragging"] and in_world(mp):
                    erase_wall_at(mp[0], mp[1], st)
                    soft_reset_path(st)
                if st["edit_mode"] == "terrain" and pygame.mouse.get_pressed()[0]:
                    paint_terrain_at(mp[0], mp[1], st)

            # ── mouse button up ────────────────────────────────────────
            if event.type == pygame.MOUSEBUTTONUP:
                if st["dragging_wall"]:
                    wp = st["wall_preview"]
                    if wp and wp.width > 0 and wp.height > 0:
                        # don't cover waypoints
                        ok = all(not wp.collidepoint(p) for _,p in st["waypoints"])
                        if ok:
                            st["walls"].append(Wall(wp.copy()))
                            soft_reset_path(st)
                    st["dragging_wall"] = False
                    st["wall_start_px"] = None
                    st["wall_preview"]  = None
                if st["erase_dragging"]:
                    st["erase_dragging"] = False

        # ── update robot position ──────────────────────────────────────
        update_robot(st, now)

        # ── draw ──────────────────────────────────────────────────────
        draw_background()
        draw_panel(st)
        draw_world_header(st["edit_mode"], st["algo"], st["diagonal"])
        draw_grid()
        draw_terrain()

        # exploration overlay
        draw_exploration(st["explored_cells"], st["frontier_cells"],
                         st["show_exploration"])

        # walls
        for w in st["walls"]:
            w.draw()

        # wall preview
        draw_wall_preview(st["wall_preview"])

        # path
        if st["path_found"]:
            draw_path_tiles(st["path_found"])

        # waypoints
        wps = st["waypoints"]
        for i, (cell, pos) in enumerate(wps):
            if i == 0:
                draw_marker(pos, START_COLOR, START_RING_COLOR, "D")
            elif i == len(wps)-1:
                draw_marker(pos, END_COLOR, END_RING_COLOR, "X")
            else:
                ci = (i-1) % len(WAYPOINT_COLORS)
                c  = WAYPOINT_COLORS[ci]
                draw_marker(pos, c, tuple(max(0,x-60) for x in c), str(i))

        # robot (smooth)
        if st["robot_path"] and (st["moving"] or st["robot_idx"] > 0):
            draw_robot(st["robot_px"], st["robot_py"])

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
