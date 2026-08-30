import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import clean_dataset


class LetterExtractionTest(unittest.TestCase):
    def test_extract_letter_ignores_green_background_at_image_edge(self):
        image = np.full((300, 400, 3), (0, 140, 0), dtype=np.uint8)
        paper = np.array([[70, 50], [330, 70], [300, 250], [95, 230]], np.int32)
        cv2.fillConvexPoly(image, paper, (255, 255, 255))
        cv2.putText(image, "A", (155, 180), cv2.FONT_HERSHEY_SIMPLEX, 3,
                    (20, 20, 20), 6, cv2.LINE_AA)

        cleaned = clean_dataset.extract_letter(image, black_threshold=180)

        self.assertIsNotNone(cleaned)
        self.assertEqual(cleaned.shape, (64, 64))
        self.assertGreater(float((cleaned > 240).mean()), 0.7)
        self.assertLess(float((cleaned < 30).mean()), 0.3)


if __name__ == "__main__":
    unittest.main()
