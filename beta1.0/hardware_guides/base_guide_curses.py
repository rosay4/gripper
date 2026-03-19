## 日志缓冲
from collections import deque
import threading
import time
import hblog
import os
import re
import textwrap
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
# 新 FPS 格式
FPS_PATTERN = re.compile(
    r"INFO.*?Step time:.*?Average FPS:\s*(?P<fps>[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)"
    r"(?:.*?\bby\s+(?P<device>[\w\-]+))?",
    re.IGNORECASE,
)

# Rate 格式
RATE_PATTERN = re.compile(
    r"INFO.*?rate=(?P<rate>[0-9]+(?:\.[0-9]+)?)(?:%|\b)(?:.*?\bby\s+(?P<device>[\w\-]+))?",
    re.IGNORECASE,
)

class HBlogBuffer:
    def __init__(self,max_lines=500):
        self.buf = deque(maxlen=max_lines)
        self.lock = threading.Lock()
    def append(self,line):
        with self.lock:
            self.buf.append(line)
    def consume(self):
        with self.lock:
            items = list(self.buf)
            self.buf.clear()
            return items

class HBlogTailer(threading.Thread):
    def __init__(self,path,buffer,poll_interval=0.1, clear_on_stop=False):
        super().__init__(daemon=True)
        self.path = path
        self.buffer = buffer
        self.poll_interval = poll_interval
        self.clear_on_stop = clear_on_stop
        self._stop = threading.Event()
    def stop(self):
        self._stop.set()
    def run(self):
        f = None
        last_inode = None
        try:
            while not self._stop.is_set():
                try:
                    st = os.stat(self.path)
                    if f is None or st.st_ino != last_inode:
                        if f:
                            f.close()
                        f = open(self.path, 'r', encoding='utf-8',errors='ignore')
                        f.seek(0, os.SEEK_END)
                        last_inode = st.st_ino
                    line = f.readline()
                    if line:
                        line = line.rstrip()
                        # 解析 FPS
                        m = FPS_PATTERN.search(line)
                        if m:
                            self.buffer.append({
                                "type": "fps",
                                "device": (m.group("device") or "gripper").strip(),
                                "fps": float(m.group("fps")),
                            })
                        # 解析 Rate
                        m = RATE_PATTERN.search(line)
                        if m:
                            self.buffer.append({
                                "type": "rate",
                                "device": (m.group("device") or "gripper").strip(),
                                "rate": float(m.group("rate")),
                            })
                        # 读原始日志
                        self.buffer.append(line)
                    else:
                        time.sleep(self.poll_interval)
                except FileNotFoundError:
                    time.sleep(0.5)
                except Exception as e:
                    self.buffer.append(f"[tail error] {e}")
                    time.sleep(1.0)
        finally:
            if f:
                try:
                    f.close()
                except Exception as e:
                    print(e)
            if self.clear_on_stop:
                try:
                    with open(self.path,'w',encodin="utf-8"):
                        pass
                except Exception as e:
                    self.buffer.append(f"[tail cleanup error] {e}")
        

class UILogger:
    def __init__(self,max_lines=500):
        self.buf = deque(maxlen=max_lines)
        self.lock = threading.Lock()
    def log(self,level,msg):
        ts = time.strftime("%H:%M:%S")
        with self.lock:
            self.buf.append(f"[{ts}]{level:<5}{msg}")
    def info(self,msg): self.log("INFO ",msg)
    def warn(self,msg): self.log("WARN ",msg)
    def error(self,msg): self.log("ERROR ",msg)
    def debug(self,msg): self.log("DEBUG ",msg)

    def snapshot(self):
        with self.lock:
            return list(self.buf)
## curses UI
import curses
import copy

class CursesUI:
    def __init__(self,guide):
        self.guide = guide
        self.logger = guide.loggerUI
        ## 布局
        self.feedback_h = 16
        self.menu_h = 14
        self.hblog_h = 8
        self.min_h = self.feedback_h + self.menu_h + self.hblog_h + 5
        self.min_w = 80
        ## 终端切换
        self.show_ui = True
        self._last_hidden_msg_time = 0.0
        self.hidden_msg_interval = 1.0
        ## hblog
        self._hblog_lines = deque(maxlen=1000)
        self.hblog_view_start = 0
        self.hblog_scroll = 0
        self.hblog_follow = True
        self._last_scroll_time = 0
        self._scroll_boost = 1
        ## 菜单分页
        self.menu_items_per_page = 8  # 每页显示8个选项
        self.menu_page = 0  # 当前页码
        self._last_menu_key = None  # 用于检测菜单变化

    def run(self):
        curses.wrapper(self._main)

    def _force_sync_term_size(self, stdscr):
        """强制同步 curses 内部尺寸，让 getmaxyx() 立刻更新"""
        try:
            tw, th = os.get_terminal_size()   # (cols, lines)
            curses.resizeterm(th, tw)         # (lines, cols)
        except Exception:
            # fallback
            try:
                curses.resize_term(0, 0)
            except Exception:
                pass

        if hasattr(curses, "update_lines_cols"):
            curses.update_lines_cols()

        stdscr.clear()
        stdscr.refresh()

    def _wait_terminal_size(self, stdscr):
        stdscr.nodelay(True)
        stdscr.keypad(True)

        while self.guide.running:
            H, W = stdscr.getmaxyx()

            stdscr.erase()
            stdscr.addstr(0, 0, "⚠️ 终端窗口太小，无法启动 UI")
            stdscr.addstr(1, 0, f"当前: {W}x{H}   需要至少: {self.min_w}x{self.min_h}")
            stdscr.addstr(2, 0, "请放大终端窗口")
            stdscr.addstr(3, 0, "按 r 或 ENTER 重试检测，按 q 退出")
            stdscr.refresh()

            if H >= self.min_h and W >= self.min_w:
                return True

            ch = stdscr.getch()

            if ch in (ord('q'), 27):
                return False

            if ch in (ord('r'), ord('R'), 10, 13):
                self._force_sync_term_size(stdscr)

            time.sleep(0.1)

        return False

    def _main(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        curses.mousemask(curses.ALL_MOUSE_EVENTS)

        # ✅ 等待终端变大，成功才继续
        ok = self._wait_terminal_size(stdscr)
        if not ok:
            return

        # ===== 进入正常 UI 初始化 =====
        H, W = stdscr.getmaxyx()
        log_h = H - self.feedback_h - self.menu_h - self.hblog_h

        self.win_feedback = curses.newwin(self.feedback_h, W, 0, 0)
        self.win_menu = curses.newwin(self.menu_h, W, self.feedback_h, 0)
        self.win_menu.nodelay(True)
        self.win_menu.keypad(True)
        self.win_log = curses.newwin(log_h, W, self.feedback_h + self.menu_h, 0)
        self.win_hblog = curses.newwin(self.hblog_h, W, self.feedback_h + self.menu_h + log_h, 0)

        stdscr.clear()
        stdscr.refresh()

        while self.guide.running:
            # ✅ 运行中如果缩小窗口，也要回到等待模式
            H2, W2 = stdscr.getmaxyx()
            if H2 < self.min_h or W2 < self.min_w:
                ok = self._wait_terminal_size(stdscr)
                if not ok:
                    return
                # ✅ 恢复后需要重新创建窗口（因为尺寸变了）
                H, W = stdscr.getmaxyx()
                log_h = H - self.feedback_h - self.menu_h - self.hblog_h
                self.win_feedback = curses.newwin(self.feedback_h, W, 0, 0)
                self.win_menu = curses.newwin(self.menu_h, W, self.feedback_h, 0)
                self.win_log = curses.newwin(log_h, W, self.feedback_h + self.menu_h, 0)
                self.win_hblog = curses.newwin(self.hblog_h, W, self.feedback_h + self.menu_h + log_h, 0)
                stdscr.clear()
                stdscr.refresh()
                continue

            # ===== 正常 UI 刷新 =====
            if self.show_ui:
                self._consume_hblog_buffer()
                self._draw_feedback()
                self._draw_menu()
                self._draw_log()
                self._draw_hblog()
                curses.doupdate()
                self.guide.motion.manual_control_1dof_step()

            self._handle_input()
            time.sleep(0.1)

    def _consume_hblog_buffer(self):
        events = self.guide.hblog_buffer.consume()
        if not events:
            return

        with self.guide.feedback_lock:
            fb = self.guide.feedbackData
            if not hasattr(fb, "device_fps"):
                fb.device_fps = {}
            if not hasattr(fb, "device_rate"):
                fb.device_rate = {}

            for item in events:
                if isinstance(item, dict):
                    if item.get("type") == "fps":
                        fb.device_fps[item["device"]] = item["fps"]
                    elif item.get("type") == "rate":
                        fb.device_rate[item["device"]] = item["rate"]
                elif isinstance(item, str):
                    self._hblog_lines.append(item)

        
    def _draw_feedback(self):
        w = self.win_feedback
        w.erase()
        w.box()
        w.addstr(0,2," Feedback ")
        w.addstr(self.feedback_h-2, 2, time.strftime("%H:%M:%S"))
        with self.guide.feedback_lock:
            fb = copy.deepcopy(self.guide.feedbackData)

        if fb is None:
            w.addstr(2,2,"No feedback yet")
        else:
            field_alias = {
                "real_distance": "real distance",
            }
            keys = list(vars(fb).keys())
            h, w_max = w.getmaxyx()
            rows_per_col = max(1, h - 3)  # keep bottom row for clock
            avail_w = max(1, w_max - 4)
            items = []
            for k in keys:
                v = getattr(fb, k)
                if k == "real_distance" and not isinstance(v, (list, tuple)):
                    v = [v]
                elif k == "gripper_limits" and isinstance(v, (list, tuple)) and len(v) >= 2:
                    try:
                        v = [round(float(v[0]), 3), round(float(v[1]), 3)]
                    except Exception:
                        pass
                label = field_alias.get(k, k)
                items.append(f"{label}:{v}")

            # Choose a column count that minimizes truncation for long fields.
            max_cols = max(1, min(len(items), avail_w // 24))
            best_cols = 1
            for cols in range(1, max_cols + 1):
                col_w = max(1, avail_w // cols)
                inner_w = max(8, col_w - 1)
                total_lines = 0
                for text in items:
                    total_lines += max(
                        1,
                        len(textwrap.wrap(text, width=inner_w, break_long_words=True, break_on_hyphens=False)),
                    )
                needed_cols = (total_lines + rows_per_col - 1) // rows_per_col
                if needed_cols <= cols:
                    best_cols = cols
                    break

            num_cols = best_cols
            col_w = max(1, avail_w // num_cols)
            inner_w = max(8, col_w - 1)

            col = 0
            y = 1
            for text in items:
                wrapped = textwrap.wrap(text, width=inner_w, break_long_words=True, break_on_hyphens=False) or [""]
                for line in wrapped:
                    if y > rows_per_col:
                        col += 1
                        y = 1
                    x = 2 + col * col_w
                    if x >= w_max - 1:
                        break
                    w.addstr(y, x, line[:inner_w])
                    y += 1
                if x >= w_max - 1:
                    break
        w.noutrefresh()
    
    def _draw_menu(self):
        w = self.win_menu
        w.erase()
        w.box()
        try:
            title, menu = self.guide.menu_stack[-1]
        except IndexError:
            title, menu = "No Menu", {}

        # 检测菜单是否变化，如果变化则重置页码
        menu_key = id(menu)
        if menu_key != self._last_menu_key:
            self.menu_page = 0
            self._last_menu_key = menu_key

        # 标题
        t = f" {title} "
        try:
            w.addstr(0, 2, t[:w.getmaxyx()[1]-4], curses.A_BOLD)
        except curses.error as e:
            print(e)

        # 计算分页
        menu_items = list(menu.items())
        total_items = len(menu_items)
        total_pages = max(1, (total_items + self.menu_items_per_page - 1) // self.menu_items_per_page)
        
        # 确保页码在有效范围内
        self.menu_page = max(0, min(self.menu_page, total_pages - 1))
        
        # 获取当前页的菜单项
        start_idx = self.menu_page * self.menu_items_per_page
        end_idx = min(start_idx + self.menu_items_per_page, total_items)
        current_page_items = menu_items[start_idx:end_idx]

        y = 1
        # 显示当前页的菜单项（重新编号为1-8，但保留原始key用于回调）
        self._current_page_key_map = {}  # 存储当前页的编号到原始key的映射
        for display_idx, (original_key, item) in enumerate(current_page_items, start=1):
            self._current_page_key_map[str(display_idx)] = original_key
            text = f"[{display_idx}] {item.get('description','')}"
            try:
                w.addstr(y, 2, text[:w.getmaxyx()[1]-4])
            except curses.error as e:
                print(e)
            y += 1

        # 返回/退出选项
        if len(self.guide.menu_stack) > 1:
            try:
                w.addstr(y, 2, "[b] 返回"[:w.getmaxyx()[1]-4])
            except curses.error as e:
                print(e)
            y += 1
        try:
            w.addstr(y, 2, "[q] 退出"[:w.getmaxyx()[1]-4])
            w.addstr(y+1, 2, "[h] 隐藏UI"[:w.getmaxyx()[1]-4])
            w.addstr(y+2, 2, "[d] 删除log文件"[:w.getmaxyx()[1]-4])
        except curses.error as e:
            print(e)

        # 显示分页信息（如果有多个页面）
        if total_pages > 1:
            page_info = f" 页 {self.menu_page + 1}/{total_pages} [↑/↓翻页] "
            try:
                w.addstr(0, w.getmaxyx()[1] - len(page_info) - 2, page_info, curses.A_DIM)
            except curses.error:
                pass

        w.noutrefresh()
    def _draw_log(self):
        w = self.win_log
        w.erase()
        w.box()
        w.addstr(0,2," Log ")
        lines = self.logger.snapshot()
        h,w_max = w.getmaxyx()
        visible = lines[-(h-2):]

        for i, line in enumerate(visible):
            w.addstr(1+i,2,line[:w_max - 4])
        
        w.noutrefresh()
    
    def _draw_hblog(self):
        w = self.win_hblog
        w.erase()
        w.box()
        title = ' hblog '
        w.addstr(0, 2, title)

        lines = list(self._hblog_lines)
        total = len(lines)
        h, w_max = w.getmaxyx()
        view_h = h - 2

        if self.hblog_follow:
            start = max(0, total - view_h)
        else:
            start = self.hblog_view_start

        end = min(total, start + view_h)
        visible = lines[start:end]

        for i, line in enumerate(visible):
            try:
                line_num = start + i + 1
                text = f"{line_num:4d} | {line}"
                w.addstr(1 + i, 2, text[:w_max - 4])
            except curses.error as e:
                print(e)

        if not self.hblog_follow:
            start_line = start + 1                # 当前视图第一行的全局行号
            end_line = min(total, start + view_h) # 当前视图最后一行的全局行号
            status = f" paused lines {start_line}-{end_line}/{total} "
            try:
                w.addstr(0, 2 + len(title) + 1, status, curses.A_DIM)
            except curses.error as e:
                print(e)


        w.noutrefresh()


    def simulate_key(self,key:str):
        """
        模拟按键输入
        """
        if key == 'h':
            self.show_ui = not self.show_ui
            if not self.show_ui:
                curses.endwin()
        if key == '\n':
            self.show_ui = True
            curses.initscr()
            curses.curs_set(0)
            H,W = curses.initscr().getmaxyx()
            log_h = H - self.feedback_h - self.menu_h - self.hblog_h
            self.win_feedback = curses.newwin(self.feedback_h, W, 0, 0)
            self.win_menu = curses.newwin(self.menu_h, W, self.feedback_h, 0)
            self.win_menu.nodelay(True)
            self.win_menu.keypad(True)
            self.win_log = curses.newwin(log_h, W, self.feedback_h + self.menu_h,0)
            self.win_hblog = curses.newwin(self.hblog_h, W, self.feedback_h + self.menu_h + log_h, 0)
            self._draw_feedback()
            self._draw_menu()
            self._draw_log()
            self._draw_hblog()
            curses.doupdate()

    def _handle_input(self):
        try:
            key = self.win_menu.getch()
            if key == -1:
                return
        except:
            return
        
        # 处理翻页按键（在转换为字符之前）
        if key == curses.KEY_UP:
            # 翻到上一页
            try:
                title, menu = self.guide.menu_stack[-1]
                total_pages = max(1, (len(menu) + self.menu_items_per_page - 1) // self.menu_items_per_page)
                if total_pages > 1:
                    self.menu_page = max(0, self.menu_page - 1)
                    self._draw_menu()
                    curses.doupdate()
            except IndexError:
                pass
            return
        
        if key == curses.KEY_DOWN:
            # 翻到下一页
            try:
                title, menu = self.guide.menu_stack[-1]
                total_pages = max(1, (len(menu) + self.menu_items_per_page - 1) // self.menu_items_per_page)
                if total_pages > 1:
                    self.menu_page = min(total_pages - 1, self.menu_page + 1)
                    self._draw_menu()
                    curses.doupdate()
            except IndexError:
                pass
            return
        
        if key == curses.KEY_MOUSE:
            self._handle_mouse()
            return

        if key > 255:
            return

        try:
            key = chr(key)
        except ValueError:
            return
        if key.lower() == 'q':
            self.guide.running = False
            return
        if key.lower() == 'b':
            self.guide.pop_menu()
            if self.show_ui:
                self._draw_menu()
                curses.doupdate()
            return
        if key.lower() == 'h':
            self.show_ui = not self.show_ui
            if not self.show_ui:
                curses.endwin()
            return
        if key.lower() == 'd':
            import shutil
            if key.lower() == 'd':
                LOG_DIR = os.path.join(project_root, "logs")
                try:
                    if not os.path.exists(LOG_DIR):
                        self.guide.loggerUI.warn(f"日志目录不存在: {LOG_DIR}")
                        return

                    for name in os.listdir(LOG_DIR):
                        path = os.path.join(LOG_DIR, name)
                        if os.path.isfile(path) or os.path.islink(path):
                            os.remove(path)
                        elif os.path.isdir(path):
                            shutil.rmtree(path)

                    self.guide.loggerUI.info(f"已清空日志目录: {LOG_DIR}")

                except Exception as e:
                    self.guide.loggerUI.error(f"清空日志目录失败: {e}")

                return
        if not self.show_ui:
            if key == '\n':
                self.show_ui = True
                curses.initscr()
                curses.curs_set(0)
                H,W = curses.initscr().getmaxyx()
                log_h = H - self.feedback_h - self.menu_h - self.hblog_h
                self.win_feedback = curses.newwin(self.feedback_h, W, 0, 0)
                self.win_menu = curses.newwin(self.menu_h, W, self.feedback_h, 0)
                self.win_menu.nodelay(True)
                self.win_menu.keypad(True)
                self.win_log = curses.newwin(log_h, W, self.feedback_h + self.menu_h,0)
                self.win_hblog = curses.newwin(self.hblog_h, W, self.feedback_h + self.menu_h + log_h, 0)
                self._draw_feedback()
                self._draw_menu()
                self._draw_log()
                self._draw_hblog()
                curses.doupdate()
            return
        title,menu = self.guide.menu_stack[-1]
        
        # 检查是否有分页映射，如果有则转换key
        original_key = key
        if hasattr(self, '_current_page_key_map') and key in self._current_page_key_map:
            original_key = self._current_page_key_map[key]
        
        if original_key in menu:
            cb = menu[original_key].get("callback")
            if callable(cb):
                try:
                    cb()
                except Exception as e:
                    self.logger.error(str(e))
                self._draw_menu()
                curses.doupdate()
    def _handle_mouse(self):
        try:
            _, _, _, _, state = curses.getmouse()
        except curses.error:
            return

        total = len(self._hblog_lines)
        view_h = self.hblog_h - 2
        max_start = max(0, total - view_h)

        now = time.time()
        if now - self._last_scroll_time < 0.15:
            self._scroll_boost = min(self._scroll_boost + 1, 5)
        else:
            self._scroll_boost = 1
        
        self._last_scroll_time = now

        SCROLL_STEP = max(1, (self.hblog_h - 2) // 4) * self._scroll_boost

        # ===== 滚轮上：看更早的日志 =====
        if state & curses.BUTTON4_PRESSED:
            if self.hblog_follow:
                # 第一次离开 follow：锁住“当前底部视图”
                self.hblog_view_start = max_start

            self.hblog_follow = False

            # 向上滚，但不小于 0
            self.hblog_view_start = max(
                0,
                self.hblog_view_start - SCROLL_STEP
            )

        # ===== 滚轮下：看更新的日志 =====
        elif state & curses.BUTTON5_PRESSED:
            if not self.hblog_follow:
                self.hblog_view_start = min(
                    max_start,
                    self.hblog_view_start + SCROLL_STEP
                )

                # 只有真正滚回到底部，才恢复 follow
                if self.hblog_view_start >= max_start:
                    self.hblog_view_start = max_start
                    self.hblog_follow = True

