
# 0,0 is left, top

def combine(all, new_part):
    for key, value in new_part.items():
        if key in all:
            all[key] = all[key] + value
        else:
            all[key] = value


front_facing = {
    "Left:Shoulder-X": ["p > Neck-X", "p > Right:Shoulder-X"],
    "Right:Shoulder-X": ["p < Neck-X", "p < Left:Shoulder-X"],
    "Left:Hip-X": ["p > Neck-X", "p > Right:Hip-X"],
    "Right:Hip-X": ["p < Neck-X", "p < Left:Hip-X"],
    "Left:Knee-X": ["p > Neck-X", "p > Right:Knee-X"],
    "Right:Knee-X": ["p < Neck-X", "p < Left:Knee-X"],
    "Left:Elbow-X": ["p > Right:Elbow-X"],
    "Right:Elbow-X": ["p < Left:Elbow-X"]
}

back_facing = {
    "Left:Shoulder-X": ["p < Neck-X"],
    "Right:Shoulder-X": ["p > Neck-X"],
    # "Left:Hip-X": ["p < Neck-X"],
    # "Right:Hip-X": ["p > Neck-X"],
    "Left:Hip-X": ["p < Right:Hip-X"],
    "Right:Hip-X": ["p > Left:Hip-X"],
    "Left:Knee-X": ["p < Neck-X"],
    "Right:Knee-X": ["p > Neck-X"]
}

side_facing = {
    "Left:Shoulder-X": ["p = Neck-X"],
    "Right:Shoulder-X": ["p = Neck-X"],
    "Left:Hip-X": ["p = Neck-X"],
    "Right:Hip-X": ["p = Neck-X"],
    "Left:Knee-X": ["p = Neck-X"],
    "Right:Knee-X": ["p = Neck-X"]
}

standing = {
    "Right:Ankle-Y": ["p = Left:Ankle-Y"],
    "Right:Knee-Y": ["p = Left:Knee-Y"],
}

right_arm_up = {
    "Right:Shoulder": ["a > 230.0"],
    "Right:Wrist-Y": ["p < Right:Elbow-Y", "p < Neck-Y"]
    }
right_arm_down = {"Right:Shoulder": ["a < 120.0"]}
right_arm_side = {"Right:Shoulder": ["a = 180.0"]}
right_arm_angled = {"Right:Elbow": ["a < 115.0"]}
right_arm_straight = {"Right:Elbow": ["a > 140.0"]}

left_arm_up = {"Left:Shoulder": ["a > 230"]}
left_arm_down = {"Left:Shoulder": ["a < 120.0"]}
left_arm_side = {"Left:Shoulder": ["a = 180.0"]}
left_arm_angled = {"Left:Elbow": ["a < 115.0"]}
left_arm_straight = {"Left:Elbow": ["a > 140.0"]}

arms_crossed_frontfacing = {
    "Left:Wrist-X": ["p < Neck-X", "p < Right:Wrist-X", "p > Right:Shoulder-X"],
    "Right:Wrist-X": ["p > Neck-X", "p > Left:Wrist-X", "p < Left:Shoulder-X"],
    "Left:Wrist-Y": ["p > Right:Shoulder-Y", "p < Right:Elbow-Y", "p = Right:Wrist-Y"],
    "Right:Wrist-Y": ["p > Left:Shoulder-Y", "p < Left:Elbow-Y", "p = Left:Wrist-Y"],
    "Left:Elbow-X": ["p > Neck-X", "p = Left:Shoulder-X"],
    "Right:Elbow-X": ["p < Neck-X", "p = Right:Shoulder-X"]
}

arms_side = {
    "Left:Wrist-X": ["p = Left:Elbow-X", "p = Left:Shoulder-X"],
    "Right:Wrist-X": ["p = Right:Elbow-X", "p = Right:Shoulder-X"],
    "Left:Elbow-X": ["p = Left:Shoulder-X"],
    "Right:Elbow-X": ["p = Right:Shoulder-X"]
}

arms_wide_frontfacing = {
    "Left:Wrist-X": ["p > Neck-X", "p > Left:Shoulder-X"],
    "Right:Wrist-X": ["p < Neck-X", "p < Right:Shoulder-X"],
    "Left:Elbow-X": ["p > Neck-X", "p > Left:Shoulder-X"],
    "Right:Elbow-X": ["p < Neck-X", "p < Right:Shoulder-X"]
}

arms_wide_backfacing = {
    "Left:Wrist-X": ["p < Neck-X", "p < Left:Shoulder-X"],
    "Right:Wrist-X": ["p > Neck-X", "p > Right:Shoulder-X"],
    "Left:Elbow-X": ["p < Neck-X", "p < Left:Shoulder-X"],
    "Right:Elbow-X": ["p > Neck-X", "p > Right:Shoulder-X"]
}

arms_left = {
    "Left:Elbow-X": ["p < Neck-X", "p < Left:Hip-X", "p < Left:Shoulder-X"],
    "Right:Elbow-X": ["p < Neck-X", "p < Right:Hip-X", "p < Right:Shoulder-X"],
    "Left:Wrist-X": ["p < Neck-X", "p < Left:Elbow-X", "p < Left:Hip-X", "p < Left:Shoulder-X"],
    "Right:Wrist-X": ["p < Neck-X", "p < Right:Elbow-X", "p < Right:Hip-X", "p < Right:Shoulder-X"]
}

arms_right = {
    "Left:Elbow-X": ["p > Neck-X", "p > Left:Hip-X", "p > Left:Shoulder-X"],
    "Right:Elbow-X": ["p > Neck-X", "p > Right:Hip-X", "p > Right:Shoulder-X"],
    "Left:Wrist-X": ["p > Neck-X", "p > Left:Elbow-X", "p > Left:Hip-X", "p > Left:Shoulder-X"],
    "Right:Wrist-X": ["p > Neck-X", "p > Right:Elbow-X", "p > Right:Hip-X", "p > Right:Shoulder-X"]
}

arms_mid_high = {
    "Left:Wrist-Y": ["p > Left:Shoulder-Y", "p < Left:Hip-Y"],
    "Right:Wrist-Y": ["p > Right:Shoulder-Y", "p < Right:Hip-Y"],
}

arms_shoulder_high = {
    "Left:Wrist-Y": ["p > Neck-Y", "p < Left:Hip-Y"],
    "Right:Wrist-Y": ["p > Neck-Y", "p < Right:Hip-Y"],
    "Left:Elbow-Y": ["p < Left:Hip-Y", "p > Neck-Y"],
    "Right:Elbow-Y": ["p < Right:Hip-Y", "p > Neck-Y"]
}

stop_wrists = {
    "Left:Wrist-Y": ["p = Left:Shoulder-Y", "p = Right:Wrist-Y"],
    "Right:Wrist-Y": ["p = Right:Shoulder-Y", "p = Left:Wrist-Y"],
    "Left:Wrist-X": ["p = Left:Shoulder-X", "p = Left:Elbow-X"],
    "Right:Wrist-X": ["p = Right:Shoulder-X", "p = Right:Elbow-X"]
}

stop_elbows = {
    "Left:Elbow-Y": ["p = Left:Shoulder-Y", "p = Right:Elbow-Y"],
    "Right:Elbow-Y": ["p = Right:Shoulder-Y", "p = Left:Elbow-Y"],
    "Left:Elbow-X": ["p = Left:Shoulder-X"],
    "Right:Elbow-X": ["p = Right:Shoulder-X"]
}

arms_down = {
    "Left:Wrist-Y": ["p > Left:Hip-Y", "p > Left:Elbow-Y"],
    "Right:Wrist-Y": ["p > Right:Hip-Y", "p > Right:Elbow-Y"]
}


gesture_call_robot = dict()
combine(gesture_call_robot, front_facing)
combine(gesture_call_robot, standing)
combine(gesture_call_robot, right_arm_up)
combine(gesture_call_robot, right_arm_straight)
combine(gesture_call_robot, left_arm_down)
combine(gesture_call_robot, left_arm_straight)

gesture_forward = dict()
combine(gesture_forward, front_facing)
combine(gesture_forward, standing)
combine(gesture_forward, right_arm_side)
combine(gesture_forward, right_arm_angled)
combine(gesture_forward, left_arm_down)
combine(gesture_forward, left_arm_straight)

gesture_backward = dict()
combine(gesture_backward, front_facing)
combine(gesture_backward, standing)
combine(gesture_backward, left_arm_side)
combine(gesture_backward, left_arm_angled)
combine(gesture_backward, right_arm_down)
combine(gesture_backward, right_arm_straight)

gesture_cancel = dict()
combine(gesture_cancel, front_facing)
combine(gesture_cancel, standing)
combine(gesture_cancel, arms_crossed_frontfacing)
combine(gesture_cancel, left_arm_down)
combine(gesture_cancel, right_arm_down)
combine(gesture_cancel, arms_mid_high)

gesture_stop = dict()
combine(gesture_stop, front_facing)
combine(gesture_stop, standing)
combine(gesture_stop, arms_side)
combine(gesture_stop, stop_wrists)
combine(gesture_stop, stop_elbows)

put_or_get_crate = dict()
combine(put_or_get_crate, back_facing)
combine(put_or_get_crate, standing)
combine(put_or_get_crate, arms_wide_backfacing)
combine(put_or_get_crate, arms_mid_high)

picking_berries_left = dict()
combine(picking_berries_left, side_facing)
combine(picking_berries_left, standing)
combine(picking_berries_left, arms_left)
combine(picking_berries_left, arms_shoulder_high)

picking_berries_right = dict()
combine(picking_berries_right, side_facing)
combine(picking_berries_right, standing)
combine(picking_berries_right, arms_right)
combine(picking_berries_right, arms_shoulder_high)

neutral = dict()
combine(neutral, front_facing)
combine(neutral, standing)
combine(neutral, arms_side)
combine(neutral, arms_down)
combine(neutral, right_arm_down)
combine(neutral, left_arm_down)
combine(neutral, right_arm_straight)
combine(neutral, left_arm_straight)

walking_away = dict()
combine(walking_away, back_facing)
combine(walking_away, arms_down)
combine(walking_away, arms_side)
combine(walking_away, right_arm_down)
combine(walking_away, left_arm_down)

walking_towards = dict()
combine(walking_towards, front_facing)
combine(walking_towards, arms_down)
combine(walking_towards, arms_side)
combine(walking_towards, right_arm_down)
combine(walking_towards, left_arm_down)

carry_away = dict()
combine(carry_away, back_facing)
combine(carry_away, arms_mid_high)
combine(carry_away, arms_wide_backfacing)
combine(carry_away, right_arm_down)
combine(carry_away, left_arm_down)

carry_towards = dict()
combine(carry_towards, front_facing)
combine(carry_towards, arms_mid_high)
combine(carry_towards, arms_wide_frontfacing)
combine(carry_towards, right_arm_down)
combine(carry_towards, left_arm_down)


# rotate_right = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 180.0", "Right:Elbow-X": "a = 180.0" }
# rotate_left = {"Left:Shoulder-X": "a = 180.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0" }
# standing = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0", "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0", "Left:Ankle-Y": "p = Right:Ankle-Y"}

# facing_towards = {"Right:Shoulder-X": "p > Left:Shoulder-X"}
# facing_away = {"Right:Shoulder-X": "p < Left:Shoulder-X"}

# pose_list = {"move forward": forward,
#              "move backward": backward,
#              "move left": rotate_right,
#              "move right": rotate_left,
#              "cancel": stop,
#              "request service": right_arm_up,
#              "picking berries left": picking_berries_left,
#              "picking berries right": picking_berries_right,
#              "facing towards": facing_towards,
#              "facing away": facing_away,
#              "has cart": has_cart}

pose_list = {"calling": gesture_call_robot,
             "picking berries": picking_berries_left,
             "picking berries right": picking_berries_right,
             # "standing": standing,
             "gesture cancel": gesture_cancel,
             "gesture stop": gesture_stop,
             "handling a crate": put_or_get_crate,
             "gesture forward": gesture_forward,
             "gesture backward": gesture_backward,
             "neutral": neutral,
             "walk_towards": walking_towards,
             "walk_away": walking_away,
             "walk_towards_crate": carry_towards,
             "walk_away_crate": carry_away,
             }

# pose_list = {"front": front_facing,
#              "back": back_facing}
