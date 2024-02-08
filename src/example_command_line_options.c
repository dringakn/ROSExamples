#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
  int opt;
  char *input_file = NULL;
  char *output_file = NULL;
  int verbose_flag = 0;
  double value = 0.0;

  static struct option long_options[] = {
      {"input", required_argument, NULL, 'i'},
      {"output", required_argument, NULL, 'o'},
      {"data", optional_argument, NULL, 'd'},
      {"verbose", no_argument, NULL, 'v'},
      {"help", no_argument, NULL, 'h'},
      {0, 0, 0, 0} // End of options
  };

  while ((opt = getopt_long(argc, argv, "i:o:d:hv", long_options, NULL)) !=
         -1) {
    switch (opt) {
    case 'i':
      input_file = optarg;
      break;
    case 'o':
      output_file = optarg;
      break;
    case 'v':
      verbose_flag = 1;
      break;
    case 'd':
      value = atof(optarg);
      break;
    case 'h':
      printf("Usage: example [OPTIONS]\n");
      printf("Options:\n");
      printf("\t-i, --input FILE    Specify input file\n");
      printf("\t-o, --output FILE   Specify output file\n");
      printf("\t-d, --data          Value\n");
      printf("\t-v, --verbose       Enable verbose mode\n");
      printf("\t-h, --help          Display this help message\n");
      exit(EXIT_SUCCESS);
    case '?':
      // Handle invalid options or missing arguments
      if (optopt == 'd') {
        value = 3.1415;
        printf("Using defalut Value:%f\n", value);
      } else {
        exit(EXIT_FAILURE);
      }
    default:
      // Handle other options
      printf("Handle other options");
      break;
    }
  }

  // Process the parsed options
  printf("Input file: %s\n", input_file);
  printf("Output file: %s\n", output_file);
  printf("Verbose mode: %s\n", verbose_flag ? "enabled" : "disabled");
  printf("Value: %f\n", value);

  return 0;
}
