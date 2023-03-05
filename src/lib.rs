#![no_std]

use core::f32::consts::{FRAC_PI_2, PI};
use core::{arch::wasm32, panic::PanicInfo};
use libm::{ceilf, cosf, fabsf, floorf, sinf, sqrtf, tanf};

const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;
const GAMEPAD1: *const u8 = 0x16 as *const u8;

const BUTTON_LEFT: u8 = 16; // 00010000
const BUTTON_RIGHT: u8 = 32; // 00100000
const BUTTON_UP: u8 = 64; // 01000000
const BUTTON_DOWN: u8 = 128; // 10000000
const STEP_SIZE: f32 = 0.045;

const FOV: f32 = PI / 2.7; // The player's field of view.
const HALF_FOV: f32 = FOV * 0.5; // Half the player's field of view.
const ANGLE_STEP: f32 = FOV / 160.0; // The angle between each ray.
const WALL_HEIGHT: f32 = 100.0; // A magic number.

const MAP: [u16; 8] = [
    0b1111111111111111,
    0b1000001010000101,
    0b1011100000110101,
    0b1000111010010001,
    0b1010001011110111,
    0b1011101001100001,
    0b1000100000001101,
    0b1111111111111111,
];

struct State {
    player_x: f32,
    player_y: f32,
    player_angle: f32,
}

static mut STATE: State = State {
    player_x: 1.5,
    player_y: 1.5,
    player_angle: 0.0,
};

extern "C" {
    fn vline(x: i32, y: i32, len: u32);
}

#[panic_handler]
fn phandler(_: &PanicInfo<'_>) -> ! {
    wasm32::unreachable();
}

#[no_mangle]
unsafe fn update() {
    STATE.update(
        *GAMEPAD1 & BUTTON_UP != 0,
        *GAMEPAD1 & BUTTON_DOWN != 0,
        *GAMEPAD1 & BUTTON_LEFT != 0,
        *GAMEPAD1 & BUTTON_RIGHT != 0,
    );

    for (x, wall) in STATE.get_view().iter().enumerate() {
        let (height, shadow) = wall;

        if *shadow {
            *DRAW_COLORS = 0x2;
        } else {
            *DRAW_COLORS = 0x3;
        }

        vline(x as i32, 80 - (height / 2), *height as u32)
    }
}

fn point_in_wall(x: f32, y: f32) -> bool {
    match MAP.get(y as usize) {
        Some(line) => (line & (0b1 << x as usize)) != 0,
        None => true,
    }
}

fn distance(a: f32, b: f32) -> f32 {
    sqrtf((a * a) + (b * b))
}

impl State {
    pub fn update(&mut self, up: bool, down: bool, left: bool, right: bool) {
        let previous_position = (self.player_x, self.player_y);

        if up {
            self.player_x += cosf(self.player_angle) * STEP_SIZE;
            self.player_y += -sinf(self.player_angle) * STEP_SIZE;
        }

        if down {
            self.player_x -= cosf(self.player_angle) * STEP_SIZE;
            self.player_y -= -sinf(self.player_angle) * STEP_SIZE;
        }

        if right {
            self.player_angle -= STEP_SIZE;
        }

        if left {
            self.player_angle += STEP_SIZE;
        }

        if point_in_wall(self.player_x, self.player_y) {
            (self.player_x, self.player_y) = previous_position;
        }
    }

    fn horizontal_insersection(&self, angle: f32) -> f32 {
        // This tell s ytou if angle if "facing up"
        // regardless of how the angle is.5
        let up = fabsf(floorf(angle / PI) % 2.0) != 0.0;

        //first_y and first_x mean how far from player to next grid intersection
        // if player is in the center of grid(0.5, 0.5) and the angle is 45, then the result （0.5，0.5）；
        // the names if very confusing
        let first_y = if up {
            ceilf(self.player_y) - self.player_y
        } else {
            floorf(self.player_y) - self.player_y
        };
        //why -first_y, going up????
        //y is flipped???
        let first_x = -first_y / tanf(angle);

        //each move
        let dy = if up { 1.0 } else { -1.0 };
        let dx = -dy / tanf(angle);

        // next_x and next_y are mutable values which will keep track
        // of how far away the ray is from the player.
        let mut next_x = first_x;
        let mut next_y = first_y;

        // This is the loop where the ray is extended until it hits
        // the wall. It's not an infinite loop as implied in the
        // explanation, instead it only goes from 0 to 256
        //
        // This was chosen becuase if seomthing goes wrong and the ray
        // never hits a wall(which should never happen) the loop will
        // eventually break and the game will keep on running.
        for _ in 0..256 {
            // current_x and current_y are where the ray is currently
            // on the map, while next_x and next_y are relative
            // coordinates, current_x and current_y are absolute
            // points.
            let current_x = next_x + self.player_x;
            let current_y = if up {
                next_y + self.player_y
            } else {
                // what does -1 mean, go up one step???
                next_y + self.player_y - 1.0
            };

            if point_in_wall(current_x, current_y) {
                break;
            }

            next_x += dx;
            next_y += dy;
        }

        distance(next_x, next_y)
    }

    fn vertical_insersection(&self, angle: f32) -> f32 {
        // This tell s ytou if angle if "facing up"
        // regardless of how the angle is.5
        let right = fabsf(floorf((angle - FRAC_PI_2) / PI) % 2.0) != 0.0;

        let first_x = if right {
            ceilf(self.player_x) - self.player_x
        } else {
            floorf(self.player_x) - self.player_x
        };
        //why -first_y, going up????
        //y is flipped???
        let first_y = -tanf(angle) * first_x;

        let dx = if right { 1.0 } else { -1.0 };
        let dy = dx * -tanf(angle);

        // next_x and next_y are mutable values which will keep track
        // of how far away the ray is from the player.
        let mut next_x = first_x;
        let mut next_y = first_y;

        // This is the loop where the ray is extended until it hits
        // the wall. It's not an infinite loop as implied in the
        // explanation, instead it only goes from 0 to 256
        //
        // This was chosen becuase if seomthing goes wrong and the ray
        // never hits a wall(which should never happen) the loop will
        // eventually break and the game will keep on running.
        for _ in 0..256 {
            // current_x and current_y are where the ray is currently
            // on the map, while next_x and next_y are relative
            // coordinates, current_x and current_y are absolute
            // points.

            let current_x = if right {
                next_x + self.player_x
            } else {
                // what does -1 mean, go up one step???
                next_x + self.player_x - 1.0
            };
            let current_y = next_y + self.player_y;

            if point_in_wall(current_x, current_y) {
                break;
            }

            next_x += dx;
            next_y += dy;
        }

        distance(next_x, next_y)
    }

    pub fn get_view(&self) -> [(i32, bool); 160] {
        // The player's FOV is split in half by their viewing angle.
        // In order to get the ray's first angle we must
        // add half the FOV to the player's angle to get
        // the edge of the player's FOV.
        let starting_angle = self.player_angle + HALF_FOV;

        let mut walls = [(0, false); 160];

        for (idx, wall) in walls.iter_mut().enumerate() {
            // 'idx' is what number ray we are, 'wall' is a mutable reference to a value in
            // 'walss'.
            let angle = starting_angle - idx as f32 * ANGLE_STEP;

            // Get both the closest horizontal and vertical wall
            // intersectiosn for this angle.

            let h_dist = self.horizontal_insersection(angle);
            let v_dist = self.vertical_insersection(angle);

            let (min_dist, shadow) = if h_dist < v_dist {
                (h_dist, false)
            } else {
                (v_dist, true)
            };

            *wall = (
                (WALL_HEIGHT / (min_dist * cosf(angle - self.player_angle))) as i32,
                shadow,
            );
        }

        walls
    }
}
