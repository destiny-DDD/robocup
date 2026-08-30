import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
import train_hog_svm


class TrainingPreprocessTest(unittest.TestCase):
    def test_higher_threshold_keeps_gray_letter_pixels(self):
        sample = Path(__file__).parent / "dataset" / "A" / "0001.png"
        normalized = train_hog_svm.normalize(sample, black_threshold=180)
        self.assertIsNotNone(normalized)
        self.assertGreater(int((normalized > 0).sum()), 100)


if __name__ == "__main__":
    unittest.main()
