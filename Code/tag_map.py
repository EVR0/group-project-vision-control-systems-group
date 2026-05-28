"""
Tag Map — maps tag IDs to physical (x, y, yaw) positions on the board.
"""

import logging
from typing import Dict, Tuple, Optional
from tag_layout import TAG_LAYOUT

log = logging.getLogger(__name__)


class TagMap:
    def __init__(self):
        self._tags: Dict[int, Tuple[float, float, float]] = dict(TAG_LAYOUT)
        log.info(f"TagMap loaded {len(self._tags)} tags")

    def get_pose(self, tag_id: int) -> Optional[Tuple[float, float, float]]:
        """Get (x, y, yaw) of a tag, or None if unknown."""
        return self._tags.get(tag_id)

    def get_all_tags(self) -> Dict[int, Tuple[float, float, float]]:
        return self._tags.copy()
