

class PointsTextFile:
    
    def __init__(self, file_name):
        """Read or write list of points coordinates from/to a text file.

        Args:
            file_name (string): Filename with extension, e.g. pointsfile.txt
        """
        self.file_name = file_name
    
    def read_points_from_text_file(self):
        """Read the points coordinates from the text file and return as list of list.

        Returns:
            points (list): List of points coordinates
        """
        pts = []
        with open(self.file_name, 'r') as f:
            for line in f:
                pts.append([float(item) for item in line.rstrip().split(',')])
                
        return pts


    def write_points_to_text_file(self, pts):
        """Write the points coordinates to the text file.

        Args:
            pts (list): List of points to be saved.
        """
        with open(self.file_name, 'w') as f:
            for line in pts:
                f.write(f"{','.join([str(x) for x in line])}\n")