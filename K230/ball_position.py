"""K230 红色位置标记与钢珠（临时圆环）位置识别。

画面约定为 640x360，导轨沿 x 轴放置；每个已知位置在导轨上下各画一条红线。
算法至少找到 -5/0/+5 cm 三组红线后，才会在导轨中寻找目标并输出毫米位置。
本文件不导入 K230 硬件模块，几何和评分逻辑可直接在 PC 上做单元测试。
"""

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360

# 红色记号笔的 LAB 初值。无需识别数字，只识别上下成对的红色短线。
# LAB 顺序为 (L_min, L_max, A_min, A_max, B_min, B_max)。
SCALE_THRESHOLDS = (
    (18, 100, 18, 127, 5, 127),
)
SCALE_SEARCH_ROI = (8, 8, 624, 344)
MARKER_INTERVAL_MM = 50.0
MARKER_MIN_COLUMNS = 2
MARKER_MAX_COLUMNS = 5
MARKER_ZERO_HINT_X = IMAGE_WIDTH * 0.5
MARKER_MIN_WIDTH = 2
MARKER_MAX_WIDTH = 36
MARKER_MIN_HEIGHT = 3
MARKER_MAX_HEIGHT = 52
MARKER_MAX_WIDTH_HEIGHT_RATIO = 2.8
MARKER_MIN_ROW_GAP_PX = 24
MARKER_MAX_ROW_GAP_PX = 180
MARKER_MAX_PAIR_X_ERROR_PX = 18
MARKER_MAX_ROW_Y_SPREAD_PX = 32
MARKER_MIN_SPACING_PX = 45
MARKER_MAX_SPACING_ERROR_RATIO = 0.32
SCALE_MIN_SPAN_RATIO = 0.52
SCALE_MAX_HEIGHT_RATIO = 0.30
SCALE_MIN_ASPECT = 4.0
SCALE_MIN_ROW_GAP_RATIO = 0.15
SCALE_MAX_ROW_GAP_RATIO = 0.80
SCALE_MIN_OVERLAP_RATIO = 0.72
SCALE_ROW_CLUSTER_HALF_HEIGHT = 22
SCALE_MIN_FRAGMENT_WIDTH = 2
SCALE_MAX_FRAGMENT_HEIGHT_RATIO = 0.20
SCALE_MIN_HORIZONTAL_COVERAGE = 0.08
SCALE_MIN_BLOB_PIXELS = 12
SCALE_MIN_BLOB_AREA = 20
SCALE_MERGE_MARGIN = 4
SCALE_REFRESH_FRAMES = 10
SCALE_HOLD_FRAMES = 30

# 样图刻度为 -12 cm 到 +12 cm；内部统一使用 mm。
SCALE_MIN_MM = -120.0
SCALE_MAX_MM = 120.0

# 临时圆环呈金黄色；钢珠则只取很暗的中性边缘，避免与灰白导轨连成一片。
RING_TARGET_THRESHOLDS = (
    (10, 90, 2, 35, 8, 50),
)
STEEL_BALL_TARGET_THRESHOLDS = (
    (0, 42, -18, 18, -18, 18),
)
TARGET_THRESHOLDS = STEEL_BALL_TARGET_THRESHOLDS
TARGET_MIN_BLOB_PIXELS = 24
TARGET_MIN_BLOB_AREA = 36
TARGET_MERGE_MARGIN = 5
TARGET_MIN_DIAMETER_PX = 7
TARGET_MAX_DIAMETER_PX = 110
TARGET_MAX_ASPECT_ERROR = 0.44
TARGET_MAX_TRACK_JUMP_PX = 110
TARGET_ROI_INSET_PX = 2


def _clip(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _blob_pixels(blob):
    try:
        return int(blob.pixels())
    except Exception:
        rect = blob.rect()
        return int(rect[2] * rect[3])


def _blob_density(blob):
    try:
        return float(blob.density())
    except Exception:
        rect = blob.rect()
        area = rect[2] * rect[3]
        return float(_blob_pixels(blob)) / float(area) if area > 0 else 0.0


class ScaleRow:
    """一排完整印刷刻度的外接矩形。"""

    __slots__ = ("x0", "x1", "center_y", "rect", "score")

    def __init__(self, x0, x1, center_y, rect, score=0.0):
        self.x0 = float(x0)
        self.x1 = float(x1)
        self.center_y = float(center_y)
        self.rect = tuple(int(value) for value in rect)
        self.score = float(score)

    @property
    def span(self):
        return self.x1 - self.x0


class DualScaleCalibration:
    """利用导轨上下两排刻度进行透视补偿的像素/毫米映射。"""

    __slots__ = ("upper", "lower", "minimum_mm", "maximum_mm", "quality")

    def __init__(
        self,
        upper,
        lower,
        minimum_mm=SCALE_MIN_MM,
        maximum_mm=SCALE_MAX_MM,
        quality=0,
    ):
        if upper.center_y >= lower.center_y:
            raise ValueError("upper scale row must be above lower scale row")
        if upper.span <= 1.0 or lower.span <= 1.0:
            raise ValueError("scale row span is too small")
        if float(maximum_mm) <= float(minimum_mm):
            raise ValueError("scale range must increase")
        self.upper = upper
        self.lower = lower
        self.minimum_mm = float(minimum_mm)
        self.maximum_mm = float(maximum_mm)
        self.quality = int(_clip(round(quality), 0, 100))

    def endpoints_at_y(self, pixel_y):
        fraction = (
            (float(pixel_y) - self.upper.center_y)
            / (self.lower.center_y - self.upper.center_y)
        )
        fraction = _clip(fraction, 0.0, 1.0)
        x0 = self.upper.x0 + fraction * (self.lower.x0 - self.upper.x0)
        x1 = self.upper.x1 + fraction * (self.lower.x1 - self.upper.x1)
        return x0, x1

    def pixel_to_mm(self, pixel_x, pixel_y=None):
        if pixel_y is None:
            pixel_y = (self.upper.center_y + self.lower.center_y) * 0.5
        x0, x1 = self.endpoints_at_y(pixel_y)
        fraction = (float(pixel_x) - x0) / (x1 - x0)
        return self.minimum_mm + fraction * (self.maximum_mm - self.minimum_mm)

    def pixels_per_mm(self, pixel_y=None):
        if pixel_y is None:
            pixel_y = (self.upper.center_y + self.lower.center_y) * 0.5
        x0, x1 = self.endpoints_at_y(pixel_y)
        return (x1 - x0) / (self.maximum_mm - self.minimum_mm)

    def target_roi(self, image_width=IMAGE_WIDTH, image_height=IMAGE_HEIGHT):
        top = int(round(self.upper.center_y + TARGET_ROI_INSET_PX))
        bottom = int(round(self.lower.center_y - TARGET_ROI_INSET_PX))
        top = int(_clip(top, 0, image_height - 1))
        bottom = int(_clip(bottom, top + 1, image_height))

        left = int(round(min(self.upper.x0, self.lower.x0)))
        right = int(round(max(self.upper.x1, self.lower.x1))) + 1
        left = int(_clip(left, 0, image_width - 1))
        right = int(_clip(right, left + 1, image_width))
        return left, top, right - left, bottom - top


class MarkerColumn:
    """同一物理刻度在导轨上下边缘形成的一对红色短线。"""

    __slots__ = (
        "upper_rect",
        "lower_rect",
        "upper_x",
        "lower_x",
        "upper_y",
        "lower_y",
        "pair_error_x",
    )

    def __init__(self, upper_rect, lower_rect):
        self.upper_rect = tuple(int(value) for value in upper_rect)
        self.lower_rect = tuple(int(value) for value in lower_rect)
        self.upper_x = self.upper_rect[0] + self.upper_rect[2] * 0.5
        self.lower_x = self.lower_rect[0] + self.lower_rect[2] * 0.5
        self.upper_y = self.upper_rect[1] + self.upper_rect[3] * 0.5
        self.lower_y = self.lower_rect[1] + self.lower_rect[3] * 0.5
        self.pair_error_x = abs(self.upper_x - self.lower_x)

    @property
    def center_x(self):
        return (self.upper_x + self.lower_x) * 0.5

    def x_at_y(self, pixel_y):
        fraction = (
            (float(pixel_y) - self.upper_y)
            / max(1.0, self.lower_y - self.upper_y)
        )
        fraction = _clip(fraction, 0.0, 1.0)
        return self.upper_x + fraction * (self.lower_x - self.upper_x)


class MarkerScaleCalibration:
    """用 -10/-5/0/5/10 cm 红色标记建立分段位置映射。"""

    __slots__ = (
        "marker_columns",
        "marker_values_mm",
        "zero_index",
        "quality",
        "marker_count",
    )

    def __init__(self, marker_columns, marker_values_mm, zero_index, quality):
        if len(marker_columns) < MARKER_MIN_COLUMNS:
            raise ValueError("need at least two red marker columns")
        if len(marker_columns) != len(marker_values_mm):
            raise ValueError("marker values must match marker columns")
        self.marker_columns = sorted(marker_columns, key=lambda column: column.center_x)
        self.marker_values_mm = [float(value) for value in marker_values_mm]
        for index in range(len(self.marker_values_mm) - 1):
            if self.marker_values_mm[index + 1] <= self.marker_values_mm[index]:
                raise ValueError("marker values must increase")
        self.zero_index = int(zero_index)
        self.quality = int(_clip(round(quality), 0, 100))
        self.marker_count = len(self.marker_columns)

    def _pixel_positions(self, pixel_y):
        return [column.x_at_y(pixel_y) for column in self.marker_columns]

    def pixel_to_mm(self, pixel_x, pixel_y=None):
        if pixel_y is None:
            pixel_y = self._rail_center_y()
        pixels = self._pixel_positions(pixel_y)
        index = 0
        while index + 2 < len(pixels) and float(pixel_x) > pixels[index + 1]:
            index += 1
        left_mm = self.marker_values_mm[index]
        interval_mm = (
            self.marker_values_mm[index + 1]
            - self.marker_values_mm[index]
        )
        fraction = (float(pixel_x) - pixels[index]) / (
            pixels[index + 1] - pixels[index]
        )
        return left_mm + fraction * interval_mm

    def pixels_per_mm(self, pixel_y=None):
        if pixel_y is None:
            pixel_y = self._rail_center_y()
        pixels = self._pixel_positions(pixel_y)
        scales = []
        for index in range(len(pixels) - 1):
            interval_mm = self.marker_values_mm[index + 1] - self.marker_values_mm[index]
            scales.append((pixels[index + 1] - pixels[index]) / interval_mm)
        return _median(scales)

    def _pixel_at_mm(self, value_mm, pixel_y):
        index = 0
        while (
            index + 2 < len(self.marker_values_mm)
            and float(value_mm) > self.marker_values_mm[index + 1]
        ):
            index += 1
        pixels = self._pixel_positions(pixel_y)
        fraction = (
            (float(value_mm) - self.marker_values_mm[index])
            / (self.marker_values_mm[index + 1] - self.marker_values_mm[index])
        )
        return pixels[index] + fraction * (pixels[index + 1] - pixels[index])

    def _rail_center_y(self):
        upper_y = sum(column.upper_y for column in self.marker_columns) / self.marker_count
        lower_y = sum(column.lower_y for column in self.marker_columns) / self.marker_count
        return (upper_y + lower_y) * 0.5

    def target_roi(self, image_width=IMAGE_WIDTH, image_height=IMAGE_HEIGHT):
        upper_y = sum(column.upper_y for column in self.marker_columns) / self.marker_count
        lower_y = sum(column.lower_y for column in self.marker_columns) / self.marker_count
        top = int(_clip(round(upper_y + TARGET_ROI_INSET_PX), 0, image_height - 1))
        bottom = int(_clip(round(lower_y - TARGET_ROI_INSET_PX), top + 1, image_height))

        center_y = (upper_y + lower_y) * 0.5
        spacing = self.pixels_per_mm(center_y) * MARKER_INTERVAL_MM
        zero_x = self._pixel_at_mm(0.0, center_y)
        left = int(_clip(round(zero_x - 2.25 * spacing), 0, image_width - 1))
        right = int(_clip(round(zero_x + 2.25 * spacing), left + 1, image_width))
        return left, top, right - left, bottom - top

    def marker_rects(self):
        rects = []
        for column in self.marker_columns:
            rects.append(column.upper_rect)
            rects.append(column.lower_rect)
        return rects


def _median(values):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def _red_marker_candidates(blobs):
    candidates = []
    for blob in blobs:
        x, y, width, height = blob.rect()
        if (
            width < MARKER_MIN_WIDTH
            or width > MARKER_MAX_WIDTH
            or height < MARKER_MIN_HEIGHT
            or height > MARKER_MAX_HEIGHT
        ):
            continue
        if float(width) / float(max(1, height)) > MARKER_MAX_WIDTH_HEIGHT_RATIO:
            continue
        candidates.append((blob, (x, y, width, height)))
    return candidates


def select_marker_calibration(blobs, image_width=IMAGE_WIDTH, image_height=IMAGE_HEIGHT):
    """把上下对齐的红色 blob 配成刻度列，中央一列自动定义为 0。"""
    del image_height
    candidates = _red_marker_candidates(blobs)
    pair_candidates = []
    for first_index in range(len(candidates)):
        first_rect = candidates[first_index][1]
        first_x = first_rect[0] + first_rect[2] * 0.5
        first_y = first_rect[1] + first_rect[3] * 0.5
        for second_index in range(first_index + 1, len(candidates)):
            second_rect = candidates[second_index][1]
            second_x = second_rect[0] + second_rect[2] * 0.5
            second_y = second_rect[1] + second_rect[3] * 0.5
            if first_y > second_y:
                upper_rect, lower_rect = second_rect, first_rect
                row_gap = first_y - second_y
            else:
                upper_rect, lower_rect = first_rect, second_rect
                row_gap = second_y - first_y
            x_error = abs(first_x - second_x)
            if not (MARKER_MIN_ROW_GAP_PX <= row_gap <= MARKER_MAX_ROW_GAP_PX):
                continue
            if x_error > MARKER_MAX_PAIR_X_ERROR_PX:
                continue
            pair_candidates.append((x_error, abs(row_gap - 70.0), first_index, second_index, upper_rect, lower_rect))

    used = set()
    columns = []
    for pair in sorted(pair_candidates, key=lambda item: (item[0], item[1])):
        first_index, second_index = pair[2], pair[3]
        if first_index in used or second_index in used:
            continue
        used.add(first_index)
        used.add(second_index)
        columns.append(MarkerColumn(pair[4], pair[5]))

    columns.sort(key=lambda column: column.center_x)
    deduplicated = []
    for column in columns:
        if deduplicated and column.center_x - deduplicated[-1].center_x < MARKER_MIN_SPACING_PX:
            if column.pair_error_x < deduplicated[-1].pair_error_x:
                deduplicated[-1] = column
        else:
            deduplicated.append(column)
    columns = deduplicated[:MARKER_MAX_COLUMNS]
    if len(columns) < MARKER_MIN_COLUMNS:
        return None

    upper_ys = [column.upper_y for column in columns]
    lower_ys = [column.lower_y for column in columns]
    if max(upper_ys) - min(upper_ys) > MARKER_MAX_ROW_Y_SPREAD_PX:
        return None
    if max(lower_ys) - min(lower_ys) > MARKER_MAX_ROW_Y_SPREAD_PX:
        return None

    spacings = [columns[index + 1].center_x - columns[index].center_x for index in range(len(columns) - 1)]
    median_spacing = _median(spacings)
    if median_spacing < MARKER_MIN_SPACING_PX:
        return None
    maximum_spacing_error = max(abs(spacing - median_spacing) for spacing in spacings)
    spacing_error_ratio = maximum_spacing_error / median_spacing
    if spacing_error_ratio > MARKER_MAX_SPACING_ERROR_RATIO:
        return None

    if len(columns) == 2:
        midpoint_x = (columns[0].center_x + columns[1].center_x) * 0.5
        nearest_zero_index = min(
            range(2),
            key=lambda index: abs(columns[index].center_x - MARKER_ZERO_HINT_X),
        )
        nearest_error = abs(columns[nearest_zero_index].center_x - MARKER_ZERO_HINT_X)
        midpoint_error = abs(midpoint_x - MARKER_ZERO_HINT_X)
        if midpoint_error < nearest_error:
            # 0 标记被钢球遮挡：可见的两组按 -5/+5 cm 处理。
            zero_index = -1
            marker_values_mm = [-MARKER_INTERVAL_MM, MARKER_INTERVAL_MM]
            estimated_zero_x = midpoint_x
        elif nearest_zero_index == 0:
            zero_index = 0
            marker_values_mm = [0.0, MARKER_INTERVAL_MM]
            estimated_zero_x = columns[0].center_x
        else:
            zero_index = 1
            marker_values_mm = [-MARKER_INTERVAL_MM, 0.0]
            estimated_zero_x = columns[1].center_x
    else:
        zero_index = min(
            range(len(columns)),
            key=lambda index: abs(columns[index].center_x - MARKER_ZERO_HINT_X),
        )
        if zero_index <= 0 or zero_index >= len(columns) - 1:
            return None
        marker_values_mm = [
            (index - zero_index) * MARKER_INTERVAL_MM
            for index in range(len(columns))
        ]
        estimated_zero_x = columns[zero_index].center_x

    pair_error = sum(column.pair_error_x for column in columns) / len(columns)
    count_score = _clip((len(columns) - 2) / 3.0, 0.0, 1.0)
    spacing_score = 1.0 - _clip(spacing_error_ratio / MARKER_MAX_SPACING_ERROR_RATIO, 0.0, 1.0)
    pair_score = 1.0 - _clip(pair_error / MARKER_MAX_PAIR_X_ERROR_PX, 0.0, 1.0)
    center_score = 1.0 - _clip(
        abs(estimated_zero_x - image_width * 0.5) / (image_width * 0.25),
        0.0,
        1.0,
    )
    quality = 100.0 * (
        0.30 * count_score
        + 0.30 * spacing_score
        + 0.25 * pair_score
        + 0.15 * center_score
    )
    if len(columns) == 2:
        quality = min(65.0, quality)
    return MarkerScaleCalibration(
        columns,
        marker_values_mm,
        zero_index,
        quality,
    )


def _interval_union_length(intervals):
    if not intervals:
        return 0.0
    ordered = sorted(intervals, key=lambda interval: interval[0])
    total = 0.0
    start = float(ordered[0][0])
    end = float(ordered[0][1])
    for next_start, next_end in ordered[1:]:
        next_start = float(next_start)
        next_end = float(next_end)
        if next_start <= end + 1.0:
            end = max(end, next_end)
        else:
            total += end - start + 1.0
            start = next_start
            end = next_end
    return total + end - start + 1.0


def _scale_fragments(blobs, image_height):
    fragments = []
    for blob in blobs:
        x, y, width, height = blob.rect()
        if width < SCALE_MIN_FRAGMENT_WIDTH or height <= 0:
            continue
        if height > float(image_height) * SCALE_MAX_FRAGMENT_HEIGHT_RATIO:
            continue
        fragments.append(
            (
                float(x),
                float(x + width - 1),
                float(y),
                float(y + height - 1),
                float(y + height * 0.5),
                float(_blob_pixels(blob)),
            )
        )
    return fragments


def _scale_row_candidate(fragments, anchor_y, image_width, image_height):
    selected = [
        fragment
        for fragment in fragments
        if abs(fragment[4] - anchor_y) <= SCALE_ROW_CLUSTER_HALF_HEIGHT
    ]
    if not selected:
        return None

    x0 = min(fragment[0] for fragment in selected)
    x1 = max(fragment[1] for fragment in selected)
    y0 = min(fragment[2] for fragment in selected)
    y1 = max(fragment[3] for fragment in selected)
    width = x1 - x0 + 1.0
    height = y1 - y0 + 1.0
    if width <= 0 or height <= 0:
        return None
    if width < float(image_width) * SCALE_MIN_SPAN_RATIO:
        return None
    if height > float(image_height) * SCALE_MAX_HEIGHT_RATIO:
        return None
    if float(width) / float(max(1, height)) < SCALE_MIN_ASPECT:
        return None

    horizontal_coverage = _interval_union_length(
        [(fragment[0], fragment[1]) for fragment in selected]
    ) / width
    if horizontal_coverage < SCALE_MIN_HORIZONTAL_COVERAGE:
        return None

    support = sum(fragment[5] for fragment in selected)
    if support <= 0.0:
        return None
    center_y = sum(fragment[4] * fragment[5] for fragment in selected) / support

    span_score = _clip(
        (float(width) / float(image_width) - SCALE_MIN_SPAN_RATIO)
        / max(0.01, 1.0 - SCALE_MIN_SPAN_RATIO),
        0.0,
        1.0,
    )
    center_error = abs((x0 + width * 0.5) - image_width * 0.5) / (image_width * 0.5)
    center_score = 1.0 - _clip(center_error, 0.0, 1.0)
    support_score = _clip(support / max(1.0, width * 2.0), 0.0, 1.0)
    coverage_score = _clip(horizontal_coverage / 0.45, 0.0, 1.0)
    score = 100.0 * (
        0.45 * span_score
        + 0.25 * center_score
        + 0.15 * support_score
        + 0.15 * coverage_score
    )
    rect = (
        int(round(x0)),
        int(round(y0)),
        int(round(width)),
        int(round(height)),
    )
    return ScaleRow(x0, x1, center_y, rect, score)


def _deduplicate_scale_candidates(candidates):
    kept = []
    for candidate in sorted(candidates, key=lambda row: row.score, reverse=True):
        duplicate = False
        for existing in kept:
            if abs(candidate.center_y - existing.center_y) <= SCALE_ROW_CLUSTER_HALF_HEIGHT:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
    return kept


def select_scale_rows(
    blobs,
    image_width=IMAGE_WIDTH,
    image_height=IMAGE_HEIGHT,
    minimum_mm=SCALE_MIN_MM,
    maximum_mm=SCALE_MAX_MM,
):
    """从蓝色长连通域中选择一对上下刻度，失败返回 ``None``。"""
    fragments = _scale_fragments(blobs, image_height)
    candidates = []
    for fragment in fragments:
        candidate = _scale_row_candidate(
            fragments,
            fragment[4],
            image_width,
            image_height,
        )
        if candidate is not None:
            candidates.append(candidate)
    candidates = _deduplicate_scale_candidates(candidates)

    best_pair = None
    best_pair_score = None
    minimum_gap = image_height * SCALE_MIN_ROW_GAP_RATIO
    maximum_gap = image_height * SCALE_MAX_ROW_GAP_RATIO

    for first_index in range(len(candidates)):
        for second_index in range(first_index + 1, len(candidates)):
            upper = candidates[first_index]
            lower = candidates[second_index]
            if upper.center_y > lower.center_y:
                upper, lower = lower, upper

            row_gap = lower.center_y - upper.center_y
            if row_gap < minimum_gap or row_gap > maximum_gap:
                continue

            overlap = min(upper.x1, lower.x1) - max(upper.x0, lower.x0)
            overlap_ratio = overlap / max(1.0, min(upper.span, lower.span))
            if overlap_ratio < SCALE_MIN_OVERLAP_RATIO:
                continue

            span_similarity = min(upper.span, lower.span) / max(upper.span, lower.span)
            pair_center_y = (upper.center_y + lower.center_y) * 0.5
            center_error = abs(pair_center_y - image_height * 0.5) / (image_height * 0.5)
            center_score = 1.0 - _clip(center_error, 0.0, 1.0)
            separation_score = _clip(
                row_gap / max(1.0, image_height * 0.35),
                0.0,
                1.0,
            )
            pair_score = 100.0 * (
                0.35 * overlap_ratio
                + 0.25 * span_similarity
                + 0.20 * center_score
                + 0.10 * separation_score
                + 0.05 * upper.score / 100.0
                + 0.05 * lower.score / 100.0
            )

            if best_pair_score is None or pair_score > best_pair_score:
                best_pair = (upper, lower)
                best_pair_score = pair_score

    if best_pair is None:
        return None
    return DualScaleCalibration(
        best_pair[0],
        best_pair[1],
        minimum_mm=minimum_mm,
        maximum_mm=maximum_mm,
        quality=best_pair_score,
    )


def detect_scale_calibration(img):
    """识别至少三列上下成对的红色记号并生成当帧标定。"""
    if img is None or img.width() != IMAGE_WIDTH or img.height() != IMAGE_HEIGHT:
        return None
    blobs = img.find_blobs(
        SCALE_THRESHOLDS,
        roi=SCALE_SEARCH_ROI,
        pixels_threshold=SCALE_MIN_BLOB_PIXELS,
        area_threshold=SCALE_MIN_BLOB_AREA,
        merge=True,
        margin=SCALE_MERGE_MARGIN,
    )
    return select_marker_calibration(blobs, img.width(), img.height())


class BallPositionResult:
    __slots__ = (
        "valid",
        "position_d10_mm",
        "center_x_d10",
        "center_y_d10",
        "quality",
        "scale_quality",
        "target_rect",
    )

    def __init__(
        self,
        valid=False,
        position_d10_mm=0,
        center_x_d10=0,
        center_y_d10=0,
        quality=0,
        scale_quality=0,
        target_rect=None,
    ):
        self.valid = bool(valid)
        self.position_d10_mm = int(position_d10_mm)
        self.center_x_d10 = int(center_x_d10)
        self.center_y_d10 = int(center_y_d10)
        self.quality = int(quality)
        self.scale_quality = int(scale_quality)
        self.target_rect = target_rect


class PiecewiseScale:
    """兼容仿真和离线标定的分段像素/毫米映射。"""

    def __init__(self, tick_pixels, tick_positions_mm):
        if len(tick_pixels) != len(tick_positions_mm) or len(tick_pixels) < 2:
            raise ValueError("need at least two pixel/scale point pairs")
        pixels = [float(value) for value in tick_pixels]
        positions = [float(value) for value in tick_positions_mm]
        if pixels[0] > pixels[-1]:
            pixels.reverse()
            positions.reverse()
        for index in range(len(pixels) - 1):
            if pixels[index + 1] <= pixels[index]:
                raise ValueError("tick pixels must be strictly monotonic")
        direction = 1.0 if positions[-1] > positions[0] else -1.0
        for index in range(len(positions) - 1):
            if (positions[index + 1] - positions[index]) * direction <= 0.0:
                raise ValueError("tick positions must be strictly monotonic")
        self.tick_pixels = pixels
        self.tick_positions_mm = positions

    def pixel_to_mm(self, pixel_x, _pipe_angle_deg=0.0):
        pixel_x = float(pixel_x)
        index = 0
        while index + 2 < len(self.tick_pixels) and pixel_x > self.tick_pixels[index + 1]:
            index += 1
        left_px = self.tick_pixels[index]
        right_px = self.tick_pixels[index + 1]
        fraction = (pixel_x - left_px) / (right_px - left_px)
        return self.tick_positions_mm[index] + fraction * (
            self.tick_positions_mm[index + 1] - self.tick_positions_mm[index]
        )


class AngleScaleBank:
    """在多组 IMU 管角标定之间插值，保留给仿真/离线标定使用。"""

    def __init__(self, angle_scale_pairs):
        if len(angle_scale_pairs) < 2:
            raise ValueError("need calibrations at two or more pipe angles")
        pairs = sorted(angle_scale_pairs, key=lambda pair: float(pair[0]))
        self.angles_deg = [float(pair[0]) for pair in pairs]
        self.scales = [pair[1] for pair in pairs]
        for index in range(len(self.angles_deg) - 1):
            if self.angles_deg[index + 1] <= self.angles_deg[index]:
                raise ValueError("calibration angles must be unique")

    def pixel_to_mm(self, pixel_x, pipe_angle_deg=0.0):
        pipe_angle_deg = float(pipe_angle_deg)
        index = 0
        while index + 2 < len(self.angles_deg) and pipe_angle_deg > self.angles_deg[index + 1]:
            index += 1
        low_angle = self.angles_deg[index]
        high_angle = self.angles_deg[index + 1]
        fraction = (pipe_angle_deg - low_angle) / (high_angle - low_angle)
        fraction = _clip(fraction, 0.0, 1.0)
        low_position = self.scales[index].pixel_to_mm(pixel_x)
        high_position = self.scales[index + 1].pixel_to_mm(pixel_x)
        return low_position + fraction * (high_position - low_position)


def select_ball_blob(blobs, previous_center_x=None, expected_diameter_px=24.0):
    """选择最像圆环/钢珠的连通域，返回 ``(blob, cx, cy, quality)``。"""
    best = None
    best_score = None
    expected_diameter_px = max(1.0, float(expected_diameter_px))

    for blob in blobs:
        x, y, width, height = blob.rect()
        if (
            width < TARGET_MIN_DIAMETER_PX
            or height < TARGET_MIN_DIAMETER_PX
            or width > TARGET_MAX_DIAMETER_PX
            or height > TARGET_MAX_DIAMETER_PX
        ):
            continue
        diameter = (width + height) * 0.5
        aspect_error = abs(width - height) / float(max(width, height))
        if aspect_error > TARGET_MAX_ASPECT_ERROR:
            continue

        center_x = x + width * 0.5
        center_y = y + height * 0.5
        if previous_center_x is not None and abs(center_x - previous_center_x) > TARGET_MAX_TRACK_JUMP_PX:
            continue

        size_error = abs(diameter - expected_diameter_px) / expected_diameter_px
        density = _blob_density(blob)
        score = (
            float(_blob_pixels(blob))
            - 100.0 * aspect_error
            - 55.0 * min(2.0, size_error)
            + 20.0 * min(1.0, density)
        )
        if previous_center_x is not None:
            score -= 0.12 * abs(center_x - previous_center_x)

        if best_score is None or score > best_score:
            quality = int(_clip(round(
                100.0
                - 95.0 * aspect_error
                - 40.0 * min(1.0, size_error)
            ), 0, 100))
            best = (blob, center_x, center_y, quality)
            best_score = score
    return best


def select_best_yolo_box(yolo_result, previous_center_x=None):
    """选择置信度最高且与上一帧连续的钢球框。

    YOLO11 视频检测结果格式为 ``(boxes, class_ids, scores)``，其中 box 为
    ``[center_x, center_y, width, height]``。返回统一后的
    ``(x, y, width, height, confidence)``。
    """
    if yolo_result is None or len(yolo_result) < 3:
        return None
    boxes = yolo_result[0]
    scores = yolo_result[2]
    best = None
    best_score = None
    count = min(len(boxes), len(scores))
    for index in range(count):
        box = boxes[index]
        if len(box) < 4:
            continue
        center_x = float(box[0])
        center_y = float(box[1])
        width = int(round(float(box[2])))
        height = int(round(float(box[3])))
        confidence = float(scores[index])
        if width <= 0 or height <= 0:
            continue
        x = int(round(center_x - width * 0.5))
        y = int(round(center_y - height * 0.5))
        continuity_penalty = 0.0
        if previous_center_x is not None:
            jump = abs(center_x - float(previous_center_x))
            continuity_penalty = jump * 0.0008
        candidate_score = confidence - continuity_penalty
        if best_score is None or candidate_score > best_score:
            best = (x, y, width, height, confidence)
            best_score = candidate_score
    return best


class BallPositionDetector:
    """先锁定双排刻度，再检测圆环/钢珠并输出相对位置。"""

    def __init__(
        self,
        scale=None,
        thresholds=None,
        roi=None,
        expected_diameter_px=24,
        target_diameter_mm=None,
        scale_refresh_frames=SCALE_REFRESH_FRAMES,
    ):
        self.scale = scale
        self.auto_scale = scale is None
        self.thresholds = TARGET_THRESHOLDS if thresholds is None else thresholds
        self.roi = roi
        self.expected_diameter_px = float(expected_diameter_px)
        self.target_diameter_mm = (
            None if target_diameter_mm is None else float(target_diameter_mm)
        )
        self.scale_refresh_frames = max(1, int(scale_refresh_frames))
        self.previous_center_x = None
        self.calibration = scale if self.auto_scale else None
        self.scale_missing_frames = 0
        self.frame_index = 0

    def reset(self):
        self.previous_center_x = None
        self.scale_missing_frames = 0
        self.frame_index = 0
        if self.auto_scale:
            self.calibration = None

    def _refresh_scale(self, img):
        should_refresh = (
            self.calibration is None
            or self.frame_index % self.scale_refresh_frames == 0
        )
        if not should_refresh:
            return self.calibration

        detected = detect_scale_calibration(img)
        if detected is not None:
            self.calibration = detected
            self.scale_missing_frames = 0
        else:
            self.scale_missing_frames += self.scale_refresh_frames
            if self.scale_missing_frames > SCALE_HOLD_FRAMES:
                self.calibration = None
                self.previous_center_x = None
        return self.calibration

    def update_calibration(self, img):
        """仅更新红色标记标定，供独立 AI 通道检测钢球时调用。"""
        if img is None or img.width() != IMAGE_WIDTH or img.height() != IMAGE_HEIGHT:
            return None
        self.frame_index += 1
        if self.auto_scale:
            return self._refresh_scale(img)
        return self.scale

    def result_from_yolo_box(self, detection):
        """把 AI 通道的钢球框中心转换成毫米位置。"""
        scale = self.calibration if self.auto_scale else self.scale
        scale_quality = 0 if scale is None else int(getattr(scale, "quality", 100))
        if detection is None:
            return BallPositionResult(scale_quality=scale_quality)

        x, y, width, height, confidence = detection
        rect = (int(x), int(y), int(width), int(height))
        center_x = float(x) + float(width) * 0.5
        center_y = float(y) + float(height) * 0.5
        target_quality = int(_clip(round(float(confidence) * 100.0), 0, 100))
        if scale is None:
            return BallPositionResult(
                quality=target_quality,
                target_rect=rect,
            )

        roi_x, roi_y, roi_width, roi_height = scale.target_roi(IMAGE_WIDTH, IMAGE_HEIGHT)
        if not (
            roi_x <= center_x <= roi_x + roi_width
            and roi_y <= center_y <= roi_y + roi_height
        ):
            return BallPositionResult(
                scale_quality=scale_quality,
                quality=target_quality,
                target_rect=rect,
            )
        if (
            self.previous_center_x is not None
            and abs(center_x - self.previous_center_x) > TARGET_MAX_TRACK_JUMP_PX
        ):
            # 大跳变先丢弃一帧并重新捕获，避免旧轨迹让新位置永久无法进入。
            self.previous_center_x = None
            return BallPositionResult(
                scale_quality=scale_quality,
                quality=target_quality,
                target_rect=rect,
            )

        self.previous_center_x = center_x
        position_mm = scale.pixel_to_mm(center_x, center_y)
        return BallPositionResult(
            valid=True,
            position_d10_mm=round(position_mm * 10.0),
            center_x_d10=round(center_x * 10.0),
            center_y_d10=round(center_y * 10.0),
            quality=min(scale_quality, target_quality),
            scale_quality=scale_quality,
            target_rect=rect,
        )

    def detect(self, img, pipe_angle_deg=0.0):
        if img is None or img.width() != IMAGE_WIDTH or img.height() != IMAGE_HEIGHT:
            return BallPositionResult()
        scale = self.update_calibration(img)
        if self.auto_scale:
            if scale is None:
                return BallPositionResult()
            target_roi = scale.target_roi(img.width(), img.height()) if self.roi is None else self.roi
            scale_quality = scale.quality
            expected_diameter_px = self.expected_diameter_px
            if self.target_diameter_mm is not None:
                expected_diameter_px = (
                    scale.pixels_per_mm() * self.target_diameter_mm
                )
        else:
            scale = self.scale
            target_roi = (0, 0, img.width(), img.height()) if self.roi is None else self.roi
            scale_quality = 100
            expected_diameter_px = self.expected_diameter_px

        blobs = img.find_blobs(
            self.thresholds,
            roi=target_roi,
            pixels_threshold=TARGET_MIN_BLOB_PIXELS,
            area_threshold=TARGET_MIN_BLOB_AREA,
            merge=True,
            margin=TARGET_MERGE_MARGIN,
        )
        selected = select_ball_blob(blobs, self.previous_center_x, expected_diameter_px)
        if selected is None:
            return BallPositionResult(scale_quality=scale_quality)

        blob, center_x, center_y, target_quality = selected
        self.previous_center_x = center_x
        if self.auto_scale:
            position_mm = scale.pixel_to_mm(center_x, center_y)
        else:
            position_mm = scale.pixel_to_mm(center_x, pipe_angle_deg)
        quality = min(int(scale_quality), int(target_quality))
        return BallPositionResult(
            valid=True,
            position_d10_mm=round(position_mm * 10.0),
            center_x_d10=round(center_x * 10.0),
            center_y_d10=round(center_y * 10.0),
            quality=quality,
            scale_quality=scale_quality,
            target_rect=blob.rect(),
        )


def format_ball_message(result, sequence):
    """UART：``B,有效,位置(0.1mm),质量,序号``。"""
    if not result.valid:
        return "B,0,0,0,{}\r\n".format(int(sequence) & 0xFFFF)
    return "B,1,{},{},{}\r\n".format(
        result.position_d10_mm,
        result.quality,
        int(sequence) & 0xFFFF,
    )
