# ── colour palette ──────────────────────────────────────────────────
CHERRY_RED   = "#D2042D"
BRIGHT_RED   = "#FF4D4D"
CREAM_BEIGE  = "#F0E1C5"
PURE_WHITE   = "#FFFFFF"
BG_COLOR     = "#0D0D0D"
DARK_NAVY    = "#0A1628"
ACCENT_GRAY  = "#333333"
LASER_GREEN  = "#39FF14"

# ── fonts ───────────────────────────────────────────────────────────
FONT_TITLE = "Noto Serif"
FONT_BODY  = "Noto Sans Mono"

# ── font sizes ──────────────────────────────────────────────────────
FS_TINY    = 16
FS_SMALL   = 20
FS_BODY    = 26
FS_HEADING = round(FS_BODY * 1.618)   # 42
FS_TITLE   = round(FS_HEADING * 1.618) # 68

# ── file paths for embedded video clips ─────────────────────────────
CONCEPT_VIDEO_1 = os.path.join("concept_1.mp4")
CONCEPT_VIDEO_2 = os.path.join("concept_2.mp4")
PROTO_VIDEO   = os.path.join("prototype.mp4")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Helpers
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _make_singapore_border() -> Polygon:
    """Create a stylised Singapore coastline polygon."""
    pts = _singapore_outline_points()
    border = Polygon(
        *pts,
        color=CREAM_BEIGE,
        stroke_width=2.5,
        fill_color=DARK_NAVY,
        fill_opacity=0.25,
    )
    border.scale(0.55)
    return border


def _laser_beam(start: np.ndarray, end: np.ndarray) -> Line:
    """Create a laser beam line from *start* to *end*."""
    return Line(
        start, end,
        color=LASER_GREEN,
        stroke_width=2,
        stroke_opacity=0.9,
    )


def _embed_video_frame(scene: Scene, video_path: str, duration: float,
                       width: float = 10.0):
    """Play an MP4 file inside the Manim scene using OpenCV frame-by-frame."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        # Fallback: show a placeholder card when the video file is missing.
        placeholder = Text(
            f"[video: {os.path.basename(video_path)}]",
            font=FONT_BODY, font_size=FS_BODY, color=ACCENT_GRAY,
        )
        scene.play(FadeIn(placeholder), run_time=0.5)
        scene.wait(duration - 0.5)
        scene.play(FadeOut(placeholder), run_time=0.3)
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    vid_duration = total_frames / fps

    # We will sample frames to match *duration* seconds at the scene rate.
    scene_fps = scene.camera.frame_rate
    num_scene_frames = int(duration * scene_fps)
    frame_indices = np.linspace(0, total_frames - 1, num_scene_frames, dtype=int)

    prev_img_mob = None
    for idx in frame_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ret, frame = cap.read()
        if not ret:
            break
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_mob = ImageMobject(frame_rgb)
        img_mob.width = width
        if prev_img_mob is not None:
            scene.remove(prev_img_mob)
        scene.add(img_mob)
        scene.wait(1 / scene_fps)
        prev_img_mob = img_mob

    if prev_img_mob is not None:
        scene.play(FadeOut(prev_img_mob), run_time=0.3)
    cap.release()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Main Scene
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class KillswitchVideoPitch(Scene):
    def construct(self):
        self.camera.background_color = BG_COLOR

        # ── Scene 1 — Hook ──────────────────────────────────────────
        self._scene_hook()

        # ── Scene 2 — Singapore map + drone swarm / laser demo ──────
        self._scene_singapore_demo()

        # ── Scene 3 — AI concept video ──────────────────────────────
        self._scene_concept_videos()

        # ── Scene 4 — Defence comparison scale ──────────────────────
        self._scene_defence_scale()

        # ── Scene 5 — Prototype demo video ──────────────────────────
        self._scene_prototype_demo()

        # ── Scene 6 — Final brand hit ───────────────────────────────
        self._scene_hook()

        self._scene_team()

    # ─────────────────────────────────────────────────────────────────
    #  Scene 1 — Hook
    # ─────────────────────────────────────────────────────────────────
    def _scene_hook(self):
        title = Text(
            "Killswitch",
            font=FONT_TITLE,
            font_size=FS_TITLE,
            color=CHERRY_RED,
        )
        subtitle = Text(
            "Low energy mobile laser air defence network",
            font=FONT_BODY,
            font_size=FS_BODY,
            color=CREAM_BEIGE,
        )
        subtitle.next_to(title, DOWN, buff=0.6)

        self.play(Write(title), run_time=1.5)
        self.play(FadeIn(subtitle, shift=UP * 0.3), run_time=1.0)
        self.wait(1.0)
        self.play(FadeOut(title), FadeOut(subtitle), run_time=0.5)

    # ─────────────────────────────────────────────────────────────────
    #  Scene 2 — Singapore map + simultaneous drone swarm / laser demo
    # ─────────────────────────────────────────────────────────────────
    def _scene_singapore_demo(self):
        # --- draw Singapore border ---
        border = _make_singapore_border()
        self.play(Create(border), run_time=1.2)

        # --- laser defence nodes along the coast of Singapore ---
        coast_positions = [
            border.get_bottom() + LEFT * 1.2 + UP * 0.9,
            border.get_bottom() + LEFT * 0.4 + UP * 0.6,
            border.get_bottom() + RIGHT * 0.4 + UP * 0.6,
            border.get_bottom() + RIGHT * 1.2 + UP * 0.9,
            border.get_left() + RIGHT * 0.3 + DOWN * 0.1,
            border.get_right() + LEFT * 0.3 + DOWN * 0.1,
        ]
        nodes = VGroup(*[
            Dot(pos, radius=0.08, color=LASER_GREEN) for pos in coast_positions
        ])
        self.play(LaggedStartMap(FadeIn, nodes, lag_ratio=0.12), run_time=0.5)

        # --- swarm of drones flying from southern coast into city centre ---
        num_drones = 15
        rng = np.random.default_rng(42)
        swarm_origin = border.get_bottom() + DOWN * 0.8
        city_centre = border.get_center() + UP * 0.2

        swarm_starts = [
            swarm_origin + np.array([rng.uniform(-1.8, 1.8), rng.uniform(-0.3, 0.3), 0])
            for _ in range(num_drones)
        ]
        swarm_ends = [
            city_centre + np.array([rng.uniform(-0.6, 0.6), rng.uniform(-0.3, 0.3), 0])
            for _ in range(num_drones)
        ]
        swarm_dots = VGroup(*[
            Dot(s, radius=0.07, color=BRIGHT_RED) for s in swarm_starts
        ])

        # threat counter
        count_label = Text(
            f"{num_drones}", font=FONT_BODY, font_size=FS_HEADING, color=CHERRY_RED,
        ).to_corner(UR, buff=0.5)
        count_title = Text(
            "THREATS", font=FONT_BODY, font_size=FS_TINY, color=CREAM_BEIGE,
        ).next_to(count_label, DOWN, buff=0.15)

        self.play(
            LaggedStartMap(FadeIn, swarm_dots, lag_ratio=0.04),
            FadeIn(count_label), FadeIn(count_title),
            run_time=0.5,
        )

        # --- simultaneously move swarm north while lasers pick them off ---
        # We step through in short increments: move drones a bit, then a
        # laser fires and eliminates one drone, count decreases.
        kill_order = list(range(num_drones))
        rng.shuffle(kill_order)
        steps = num_drones
        alive_set = set(range(num_drones))

        for step_i in range(steps):
            # move all alive drones a fraction toward their destination
            frac = (step_i + 1) / steps / 2
            move_anims = []
            for di in alive_set:
                target_pos = (
                    np.array(swarm_starts[di]) * (1 - frac)
                    + np.array(swarm_ends[di]) * frac
                )
                move_anims.append(swarm_dots[di].animate.move_to(target_pos))

            # pick one to kill
            victim_idx = kill_order[step_i]
            shooter = nodes[step_i % len(nodes)]
            beam = _laser_beam(shooter.get_center(), swarm_dots[victim_idx].get_center())

            remaining = num_drones - step_i - 1
            new_count = Text(
                f"{remaining}", font=FONT_BODY, font_size=FS_HEADING,
                color=CHERRY_RED,
            ).move_to(count_label)

            self.play(
                *move_anims,
                Create(beam),
                swarm_dots[victim_idx].animate.set_opacity(0),
                Transform(count_label, new_count),
                run_time=0.18,
                rate_func=linear,
            )
            self.remove(beam)
            alive_set.discard(victim_idx)

        self.wait(0.3)

        # --- slide map left, show bullet points ---
        map_group = VGroup(border, nodes, swarm_dots)
        self.play(
            map_group.animate.scale(0.7).shift(LEFT * 3.0),
            FadeOut(count_label), FadeOut(count_title),
            run_time=0.8,
        )

        bullets = VGroup(
            Text("• cost effective: <$3 / drone", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("• highly mobile", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("• virtually infinite magazine", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.35)
        bullets.next_to(ORIGIN, RIGHT, buff=0.5)

        self.play(LaggedStartMap(FadeIn, bullets, shift=RIGHT * 0.3,
                                 lag_ratio=0.3), run_time=1.2)
        self.wait(1.0)

        # clean up
        self.play(
            *[FadeOut(m) for m in self.mobjects],
            run_time=0.5,
        )

    # ─────────────────────────────────────────────────────────────────
    #  Scene 3 — AI concept video (via OpenCV)
    # ─────────────────────────────────────────────────────────────────
    def _scene_concept_videos(self):
        t_concept_1 = Text("Ground implementation", font=FONT_TITLE, font_size=FS_HEADING, color=PURE_WHITE)
        t_concept_2 = Text("Rooftop implementation", font=FONT_TITLE, font_size=FS_HEADING, color=PURE_WHITE)
        
        self.play(Write(t_concept_1), run_time=1.0)
        _embed_video_frame(self, CONCEPT_VIDEO_1, duration=4.0, width=11)
        
        self.play(Transform(t_concept_1, t_concept_2))
        _embed_video_frame(self, CONCEPT_VIDEO_2, duration=4.0, width=11)
        
        # clean up
        self.play(*[FadeOut(m) for m in self.mobjects], run_time=0.5)


    # ─────────────────────────────────────────────────────────────────
    #  Scene 4 — Defence comparison scale
    # ─────────────────────────────────────────────────────────────────
    def _scene_defence_scale(self):
        # --- draw the scale bar ---
        scale_line = Line(LEFT * 5, RIGHT * 5, color=CREAM_BEIGE, stroke_width=3)
        left_label = Text(
            "DRONE\nDEFENCE", font=FONT_BODY, font_size=FS_SMALL, color=CREAM_BEIGE,
        ).next_to(scale_line.get_left(), DOWN, buff=0.4)
        right_label = Text(
            "MISSILE\nINTERCEPTOR", font=FONT_BODY, font_size=FS_SMALL, color=CREAM_BEIGE,
        ).next_to(scale_line.get_right(), DOWN, buff=0.4)

        # ticks
        left_tick  = Line(UP * 0.15, DOWN * 0.15, color=CREAM_BEIGE, stroke_width=2)\
            .move_to(scale_line.get_left())
        right_tick = Line(UP * 0.15, DOWN * 0.15, color=CREAM_BEIGE, stroke_width=2)\
            .move_to(scale_line.get_right())

        scale_group = VGroup(scale_line, left_label, right_label, left_tick, right_tick)
        scale_group.shift(UP * 1.5)

        self.play(
            Create(scale_line),
            FadeIn(left_label), FadeIn(right_label),
            Create(left_tick), Create(right_tick),
            run_time=1.0,
        )

        # --- annotation labels above each end ---
        insufficient_label = Text(
            "INSUFFICIENT", font=FONT_BODY, font_size=FS_TINY,
            color=BRIGHT_RED, weight="BOLD",
        ).next_to(left_label, UP, buff=0.8)
        overkill_label = Text(
            "OVERKILL", font=FONT_BODY, font_size=FS_TINY,
            color=BRIGHT_RED, weight="BOLD",
        ).next_to(right_label, UP, buff=0.8)

        self.play(FadeIn(insufficient_label), FadeIn(overkill_label), run_time=0.6)

        # --- highlight LASER region in the middle ---
        highlight = Rectangle(
            width=3.5, height=0.6,
            color=CHERRY_RED, fill_color=CHERRY_RED, fill_opacity=0.2,
            stroke_width=2,
        ).move_to(scale_line.get_center())
        laser_label = Text(
            "LASER", font=FONT_TITLE, font_size=FS_HEADING,
            color=CHERRY_RED, weight="BOLD",
        ).next_to(highlight, DOWN, buff=0.3)

        self.play(FadeIn(highlight), Write(laser_label), run_time=1.0)
        self.wait(0.5)

        # --- slide scale up, show laser benefits ---
        top_group = VGroup(scale_group, highlight, laser_label, insufficient_label, overkill_label)
        self.play(top_group.animate.shift(UP * 0.8).scale(0.8), run_time=0.6)

        benefits = VGroup(
            Text("5 kW", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("max 7 s dwell time", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("<$3 / drone threat", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("minimal collateral damage", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("target: swarm of small drones", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("optimum weather: clear sky", font=FONT_BODY,
                 font_size=FS_SMALL, color=PURE_WHITE),
            Text("forces adversary to deploy in bad conditions", font=FONT_BODY,
                 font_size=FS_SMALL*0.618, color=PURE_WHITE),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.3)
        benefits.next_to(top_group, DOWN, buff=0.6)

        self.play(
            LaggedStartMap(FadeIn, benefits, shift=RIGHT * 0.3, lag_ratio=0.25),
            run_time=2.5,
        )
        self.wait(2.0)

        # clean up
        self.play(*[FadeOut(m) for m in self.mobjects], run_time=0.5)

    # ─────────────────────────────────────────────────────────────────
    #  Scene 5 — Prototype demo video (via OpenCV)
    # ─────────────────────────────────────────────────────────────────
    def _scene_prototype_demo(self):
        _embed_video_frame(self, PROTO_VIDEO, duration=4.0, width=11)

    def _scene_team(self):
        BOX_W, BOX_H = 2.2, 2.8

        team_data = [
            ("Aadith Yadav G", "Simulations & video", "Material Science Engineer", "I-FIM, SafeRail.AI", "aadith.jpg"),
            ("Rohit Panda", "Software", "AI Engineer", "Masters", "rohit.jpg"),
            ("Evan Tok", "Hardware", "Electrical Power Engineer", "Siemens Energy", "evan.jpg"),
            ("Thejus Aravind", "Video & pitch", "Mechanical Engineer", "NUS Mars Rover", "thejus.jpg"),
        ]

        photo_group = Group()
        target_aspect = BOX_W / BOX_H
        for _, _, _, _, filename in team_data:
            # Center-crop image to target aspect ratio using PIL
            pil_img = Image.open(filename)
            iw, ih = pil_img.size
            img_aspect = iw / ih
            if img_aspect > target_aspect:
                # Too wide — crop sides
                new_w = int(ih * target_aspect)
                left = (iw - new_w) // 2
                pil_img = pil_img.crop((left, 0, left + new_w, ih))
            else:
                # Too tall — crop top/bottom
                new_h = int(iw / target_aspect)
                top = (ih - new_h) // 2
                pil_img = pil_img.crop((0, top, iw, top + new_h))
            # Save cropped image to temp file and load into Manim
            tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
            pil_img.save(tmp.name)
            img = ImageMobject(tmp.name)
            img.height = BOX_H
            border = Rectangle(width=BOX_W, height=BOX_H, color=CREAM_BEIGE, stroke_width=2, stroke_opacity=0.5)
            border.move_to(img.get_center())
            photo_group.add(Group(img, border))

        photo_group.arrange(RIGHT, buff=1.0).move_to(UP * 0.5)

        profiles = Group()
        text_groups = VGroup()
        for i, (name, role, study, work, _) in enumerate(team_data):
            n = Text(name, font=FONT_BODY, font_size=FS_SMALL, weight=BOLD, color=PURE_WHITE)
            r = Text(role, font=FONT_BODY, font_size=FS_TINY, color=CREAM_BEIGE)
            s = Text(study, font=FONT_BODY, font_size=FS_TINY*0.618, color=CREAM_BEIGE)
            w = Text(work, font=FONT_BODY, font_size=FS_TINY*0.618, color=CREAM_BEIGE)
            txt = VGroup(n, r, s, w).arrange(DOWN, buff=0.1).next_to(photo_group[i], DOWN, buff=0.3)
            text_groups.add(txt)
            profiles.add(Group(photo_group[i], txt))

        # FadeIn photos first, then Write text labels
        self.play(FadeIn(photo_group, lag_ratio=0.2), run_time=1.0)
        self.play(*[Write(txt, run_time=1.2) for txt in text_groups], lag_ratio=0.3)
        self.wait(2)

        self.play(FadeOut(profiles))
