#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

void inverse_matrix(const gsl_matrix *input, gsl_permutation *p, gsl_matrix *output)
{
    gsl_matrix *lu_decomp = gsl_matrix_alloc(input->size1, input->size2);

    gsl_matrix_memcpy(lu_decomp, input);

    int signum;
    gsl_linalg_LU_decomp(lu_decomp, p, &signum);
    gsl_linalg_LU_invert(lu_decomp, p, output);

    gsl_matrix_free(lu_decomp);
}

int main()
{
    double input_data[6][6] = {
        {1.0, 2.0, 0.0, 3.0, 0.0, 1.0},
        {0.0, 1.0, 0.0, 0.0, 2.0, 0.0},
        {0.0, 0.0, 3.0, 0.0, 0.0, 0.0},
        {4.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0, 5.0, 0.0},
        {0.0, 0.0, 0.0, 2.0, 0.0, 1.0}};

    gsl_matrix_view input = gsl_matrix_view_array((double *)input_data, 6, 6);
    gsl_matrix *output = gsl_matrix_alloc(6, 6);
    gsl_permutation *p = gsl_permutation_alloc(6);
    gsl_permutation_init(p);

    inverse_matrix(&input.matrix, p, output);

    printf("Inverse Matrix:\n");
    for (size_t i = 0; i < 6; ++i)
    {
        for (size_t j = 0; j < 6; ++j)
        {
            printf("%6.3f ", gsl_matrix_get(output, i, j));
        }
        printf("\n");
    }

    gsl_matrix_free(output);
    gsl_permutation_free(p);

    return 0;
}
