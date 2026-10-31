import base64
import struct
import unittest
import zlib

from package import APP_SIZE, APP_START, RAM_END, make_package


def image_with_vectors(stack=RAM_END, entry=APP_START + 17):
    return struct.pack("<II", stack, entry) + bytes(range(41))


class PackageTests(unittest.TestCase):
    def test_container_preserves_image_and_aligns_h7_flash_words(self):
        image = image_with_vectors()
        package = make_package(image)
        recovered = zlib.decompress(base64.b64decode(package["image"]))

        self.assertEqual(recovered[:len(image)], image)
        self.assertEqual(recovered[len(image):], b"\xff" * (64 - len(image)))
        self.assertEqual(package["image_size"], len(recovered))
        self.assertEqual(package["board_id"], 53)

    def test_rejects_direct_boot_or_non_thumb_reset_vectors(self):
        for entry in [0x08000011, APP_START + 16, APP_START + 0x10001]:
            with self.subTest(entry=entry), self.assertRaises(ValueError):
                make_package(image_with_vectors(entry=entry))

    def test_rejects_invalid_stack_and_truncated_images(self):
        for stack in [0x20020000, RAM_END + 8, RAM_END - 1]:
            with self.subTest(stack=stack), self.assertRaises(ValueError):
                make_package(image_with_vectors(stack=stack))
        with self.assertRaises(ValueError):
            make_package(b"\x00" * 7)

    def test_rejects_image_exceeding_application_partition(self):
        image = image_with_vectors() + b"\xff" * APP_SIZE
        with self.assertRaises(ValueError):
            make_package(image)


if __name__ == "__main__":
    unittest.main()
