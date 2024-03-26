#!/usr/bin/env python3

import os
import argparse
from PIL import Image


class AprilTagConverter:
    def __init__(self):
        self.parser = self._create_parser()

    def _create_parser(self):
        parser = argparse.ArgumentParser(
            description='A script to convert pre-generated apriltag .png files into SVG format.',
            epilog='Example: "python3 tag_to_svg.py --input=./tag36h11/tag36_11_00000.png --output=./tag36h11_50mm/tag36_11_00000.svg --size=50mm" or "python3 tag_to_svg.py --input=./tag36h11 --output=./tag36h11_50mm --size=50mm"'
        )
        parser.add_argument(
            '--input', type=self._validate_path, required=True,
            help='The path to the input file or folder containing apriltag png files to convert.'
        )
        parser.add_argument(
            '--output', type=str, required=True,
            help='The path to the output folder where SVG files will be saved.'
        )
        parser.add_argument(
            '--size', type=str, required=False, default='20mm', dest="svg_size",
            help='The size (edge length) of the generated svg such as "20mm" "2in" "20px"'
        )
        return parser

    @staticmethod
    def _validate_path(path):
        if os.path.exists(path):
            return path
        else:
            raise argparse.ArgumentTypeError(f'Supplied argument "{path}" does not exist.')

    @staticmethod
    def _generate_rgba(rbga):
        (_r, _g, _b, _raw_a) = rbga
        _a = _raw_a / 255
        return f'rgba({_r}, {_g}, {_b}, {_a})'

    def _generate_gridsquare(self, row_num, col_num, pixel):
        _rgba = self._generate_rgba(pixel)
        _id = f'box{row_num}-{col_num}'
        return f'\t<rect width="1" height="1" x="{row_num}" y="{col_num}" fill="{_rgba}" id="{_id}"/>\n'

    def _generate_apriltag_svg(self, width, height, pixel_array, size,
                               border_width=1, border_color="black",
                               image_name=None):
        svg_text = '<?xml version="1.0" standalone="yes"?>\n'
        svg_text += f'<svg width="{size}" height="{size}" viewBox="-{border_width} -{border_width} {width + 2 * border_width} {height + 2 * border_width}" xmlns="http://www.w3.org/2000/svg">\n'
        # Fill the area between border and image with a solid color
        svg_text += f'\t<rect width="{width + 2 * border_width}" height="{height + 2 * border_width}" x="-{border_width}" y="-{border_width}" fill="white" />\n'
        # Generate border rectangle
        svg_text += f'\t<rect width="{width + 2 * border_width}" height="{height + 2 * border_width}" x="-{border_width}" y="-{border_width}" fill="none" stroke="{border_color}" stroke-width="{border_width}" />\n'
        for _y in range(height):
            for _x in range(width):
                svg_text += self._generate_gridsquare(_x, _y, pixel_array[_x, _y])
        # Add image name text at the bottom
        if image_name:
            text_x = width / 2
            text_y = height
            svg_text += f'\t<text x="{text_x}" y="{text_y}" text-anchor="middle" fill="{border_color}" font-size="1" font-family="sans-serif">{image_name}</text>\n'
        svg_text += '</svg>\n'
        return svg_text

    def _create_output_folder(self, output_folder):
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            print(f'Created output folder: {output_folder}')

    def convert_to_svg(self):
        args = self.parser.parse_args()
        input_path = args.input
        output_folder = args.output
        svg_size = args.svg_size

        self._create_output_folder(output_folder)

        if os.path.isfile(input_path):
            self._convert_single_file(input_path, output_folder, svg_size)
        elif os.path.isdir(input_path):
            self._convert_folder(input_path, output_folder, svg_size)
        else:
            print(f"Error: Invalid input path {input_path}")

    def _convert_single_file(self, input_file, output_folder, svg_size):
        if not input_file.lower().endswith('.png'):
            print(f"Error: {input_file} is not a PNG file.")
            return

        output_file = os.path.join(output_folder, os.path.basename(input_file).replace('.png', '.svg'))

        with Image.open(input_file, 'r') as im:
            width, height = im.size
            pix_vals = im.load()
            apriltag_svg = self._generate_apriltag_svg(width, height, pix_vals, svg_size,
                                                       1, 'black', None)

        with open(output_file, 'w') as fp:
            fp.write(apriltag_svg)

        print(f'Converted {input_file} to SVG: {output_file} with size: {svg_size}')

    def _convert_folder(self, input_folder, output_folder, svg_size):
        for filename in os.listdir(input_folder):
            if filename.lower().endswith(".png"):
                input_file_path = os.path.join(input_folder, filename)
                output_file_path = os.path.join(output_folder, filename.replace(".png", ".svg"))

                with Image.open(input_file_path, 'r') as im:
                    width, height = im.size
                    pix_vals = im.load()
                    apriltag_svg = self._generate_apriltag_svg(width, height, pix_vals, svg_size,
                                                               1, 'black', None)

                with open(output_file_path, 'w') as fp:
                    fp.write(apriltag_svg)

                print(f'Converted {input_file_path} to SVG: {output_file_path} with size: {svg_size}')


def main():
    converter = AprilTagConverter()
    converter.convert_to_svg()


if __name__ == "__main__":
    main()
