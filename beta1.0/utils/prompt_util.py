
from collections.abc import Callable
from typing import Any

from prompt_toolkit import prompt
from prompt_toolkit.application.current import get_app
from prompt_toolkit.formatted_text import FormattedText
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.keys import Keys

OptionType = list[tuple[str, str] | str]


def select_options(
    prompttext: str,
    options: OptionType,
    start: int = 0,
    head: str | None = None,
    callbacks: (
        dict[str, Callable[[int], tuple[int, bool]]] | None
    ) = None,  # return index, and if we need to update the options
    update_fn: Callable[[], tuple[OptionType, str | None]] | None = None,
) -> tuple[int, Any]:
    """
    When callback return index:
        - refresh the options
        - and continue the loop
    Otherwise, exit the app
    """

    selected_index: int = start
    start = 0
    end = None

    bindings = KeyBindings()

    @bindings.add(Keys.Down)
    def _(event):
        nonlocal selected_index
        selected_index = (selected_index + 1) % len(options)
        event.app.layout.current_buffer.reset()

    @bindings.add(Keys.Up)
    def _(event):
        nonlocal selected_index
        selected_index = (selected_index - 1) % len(options)
        event.app.layout.current_buffer.reset()

    callback_keys = None
    quit_without_select = False

    @bindings.add(Keys.Any)
    def _(event):
        nonlocal options, head
        key = event.key_sequence[0].key

        if "0" <= key <= "9":
            nonlocal selected_index

            index = int(key) - start
            if 0 <= index < len(options):
                event.app.exit()
                selected_index = index

        if key == "q":
            nonlocal quit_without_select
            quit_without_select = True
            event.app.exit()

        if callbacks is not None and key in callbacks:
            nonlocal callback_keys
            callback_keys = key
            event.app.exit()

    def get_prompt_text():
        nonlocal start, end
        height = get_app().output.get_size().rows
        end = start + height - 2
        while selected_index >= end:
            start += 1
            end += 1
        while selected_index < start:
            start -= 1
            end -= 1
        height = min(height, len(options))
        headline = f" {prompttext} (~{height}/{len(options)}):  <-- Use arrow keys or numbers to select an option or press 'q' to exit.\n"
        if head is not None:
            headline += head + "\n"
        formatted_options: list[tuple[str, str]] = [("default", headline)]
        for i, option in enumerate(options):
            default = "default"
            if isinstance(option, tuple):
                option, default = option
            if i >= start and i < end:
                if i == selected_index:
                    formatted_options.append(("green", f"> {i + start:3}. {option}\n"))
                else:
                    formatted_options.append((default, f"  {i + start:3}. {option}\n"))
        return FormattedText(formatted_options)

    while True:
        prompt(get_prompt_text, key_bindings=bindings)

        if callback_keys is not None and callbacks is not None:
            new_index, need_update = callbacks[callback_keys](selected_index)
            if need_update:
                assert (
                    update_fn is not None
                ), f"update_fn is required by callbacks with need_update=True for {callback_keys}"
                options, head = update_fn()
                selected_index = max(new_index, 0)

            callback_keys = None
            continue
        else:
            break

    if quit_without_select:
        return -1, None
    return selected_index, options[selected_index]
