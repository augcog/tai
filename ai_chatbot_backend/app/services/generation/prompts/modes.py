"""
4-Mode System Configuration.

This module provides the Mode enum, ModeConfig dataclass, and functions
to get mode configurations. The actual prompts live in the textchat/
and voice/ subfolders for easy editing.

Each mode file exports two complete prompts (WITH_REFS and NO_REFS).
Only {course} and {class_name} are resolved at runtime via .format().

4-Mode System:
- TEXT_CHAT_TUTOR: tutor_mode=True, audio_response=False
- TEXT_CHAT_REGULAR: tutor_mode=False, audio_response=False
- VOICE_TUTOR: tutor_mode=True, audio_response=True
- VOICE_REGULAR: tutor_mode=False, audio_response=True
"""

from dataclasses import dataclass
from enum import Enum, auto

# Import complete prompts from subfolders
from app.services.generation.prompts.textchat import (
    TUTOR_PROMPT_WITH_REFS as TEXT_CHAT_TUTOR_WITH_REFS,
    TUTOR_PROMPT_NO_REFS as TEXT_CHAT_TUTOR_NO_REFS,
    REGULAR_PROMPT_WITH_REFS as TEXT_CHAT_REGULAR_WITH_REFS,
    REGULAR_PROMPT_NO_REFS as TEXT_CHAT_REGULAR_NO_REFS,
    OUTLINE_PROMPT_WITH_REFS as TEXT_OUTLINE_TUTOR_WITH_REFS,
    OUTLINE_PROMPT_NO_REFS as TEXT_OUTLINE_TUTOR_NO_REFS,
)
from app.services.generation.prompts.voice import (
    TUTOR_PROMPT_WITH_REFS as VOICE_TUTOR_WITH_REFS,
    TUTOR_PROMPT_NO_REFS as VOICE_TUTOR_NO_REFS,
    REGULAR_PROMPT_WITH_REFS as VOICE_REGULAR_WITH_REFS,
    REGULAR_PROMPT_NO_REFS as VOICE_REGULAR_NO_REFS,
)


class Mode(Enum):
    """Enumeration of the operating modes."""
    TEXT_CHAT_TUTOR = auto()    # tutor_mode=True, audio_response=False
    TEXT_CHAT_REGULAR = auto()  # tutor_mode=False, audio_response=False
    VOICE_TUTOR = auto()        # tutor_mode=True, audio_response=True
    VOICE_REGULAR = auto()      # tutor_mode=False, audio_response=True
    TEXT_OUTLINE_TUTOR = auto() # outline mode for tutor


@dataclass(frozen=True)
class ModeConfig:
    """Complete configuration for a mode — two self-contained prompts."""
    mode: Mode
    system_prompt_with_refs: str   # complete prompt when references exist
    system_prompt_no_refs: str     # complete prompt when no references
    is_tutor: bool
    is_audio: bool


# Registry - single source of truth for all mode configurations
MODE_CONFIGS = {
    Mode.TEXT_CHAT_TUTOR: ModeConfig(
        mode=Mode.TEXT_CHAT_TUTOR,
        system_prompt_with_refs=TEXT_CHAT_TUTOR_WITH_REFS,
        system_prompt_no_refs=TEXT_CHAT_TUTOR_NO_REFS,
        is_tutor=True,
        is_audio=False,
    ),
    Mode.TEXT_CHAT_REGULAR: ModeConfig(
        mode=Mode.TEXT_CHAT_REGULAR,
        system_prompt_with_refs=TEXT_CHAT_REGULAR_WITH_REFS,
        system_prompt_no_refs=TEXT_CHAT_REGULAR_NO_REFS,
        is_tutor=False,
        is_audio=False,
    ),
    Mode.VOICE_TUTOR: ModeConfig(
        mode=Mode.VOICE_TUTOR,
        system_prompt_with_refs=VOICE_TUTOR_WITH_REFS,
        system_prompt_no_refs=VOICE_TUTOR_NO_REFS,
        is_tutor=True,
        is_audio=True,
    ),
    Mode.VOICE_REGULAR: ModeConfig(
        mode=Mode.VOICE_REGULAR,
        system_prompt_with_refs=VOICE_REGULAR_WITH_REFS,
        system_prompt_no_refs=VOICE_REGULAR_NO_REFS,
        is_tutor=False,
        is_audio=True,
    ),
    Mode.TEXT_OUTLINE_TUTOR: ModeConfig(
        mode=Mode.TEXT_OUTLINE_TUTOR,
        system_prompt_with_refs=TEXT_OUTLINE_TUTOR_WITH_REFS,
        system_prompt_no_refs=TEXT_OUTLINE_TUTOR_NO_REFS,
        is_tutor=True,
        is_audio=False,
    ),
}


def get_mode(tutor_mode: bool, audio_response: bool) -> Mode:
    """Map boolean flags to Mode enum."""
    if audio_response:
        return Mode.VOICE_TUTOR if tutor_mode else Mode.VOICE_REGULAR
    else:
        return Mode.TEXT_CHAT_TUTOR if tutor_mode else Mode.TEXT_CHAT_REGULAR


def get_mode_config(tutor_mode: bool, audio_response: bool) -> ModeConfig:
    """
    Get complete mode configuration in one call.

    This is the main entry point for getting all prompts for a mode.
    Replaces scattered if/else logic with a single lookup.
    """
    mode = get_mode(tutor_mode, audio_response)
    return MODE_CONFIGS[mode]


def get_outline_mode_config() -> ModeConfig:
    """Get the outline tutor mode configuration."""
    return MODE_CONFIGS[Mode.TEXT_OUTLINE_TUTOR]


def get_complete_system_prompt(
    tutor_mode: bool,
    audio_response: bool,
    has_refs: bool,
    course: str = "",
    class_name: str = ""
) -> str:
    """
    Get the complete prompt the model receives, with {course}/{class_name} resolved.

    Useful for debugging and testing.
    """
    config = get_mode_config(tutor_mode, audio_response)
    prompt = config.system_prompt_with_refs if has_refs else config.system_prompt_no_refs
    return prompt.format(course=course, class_name=class_name)


def get_system_prompt(tutor_mode: bool, audio_response: bool) -> str:
    """
    Get the default system prompt for the specified mode (no_refs variant).

    Used by format_chat_msg() for the initial system message before
    references are determined. Will be replaced later in the pipeline.
    """
    return get_mode_config(tutor_mode, audio_response).system_prompt_no_refs
