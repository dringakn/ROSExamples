# import scipy.stats
# import numpy as np

# # Your data: number of candies in each bag
# data = [8, 9, 7, 10, 8, 9, 10, 11, 7, 12]
# data = np.random.randint(1, 100, (100,1))
# print(data.tolist())
# n = len(data)

# # Degrees of freedom
# df = n - 1

# # Probability value for 70%
# p = 0.999

# # Calculate the critical value
# critical_value = scipy.stats.chi2.ppf(p, df)

# print(f"Number of candies higher than 70% of the counts: {critical_value}")

'''
---------------------------------------------------------------------------------------
'''

import numpy as np
from scipy.stats import chi2_contingency

# Step 2: Collect and Organize Data
# Define the observed data as a 2D array (contingency table)
'''
Survey about favorite ice cream flavors among two different age groups:
"Young Adults" (18-30 years old) and "Middle-Aged Adults" (31-50 years old).

    Null Hypothesis (H₀):
        There is no association between age group and favorite ice cream flavor.
    Alternative Hypothesis (H₁):
        There is an association between age group and favorite ice cream flavor.

Organized collected survey data:

Favoruite Icecream  Chocolate	Vanilla	Strawberry	Other   Total
Young(18-30)	    60	        45	    30	        65      200
Middle-Aged(31-50)	40	        30	    40	        40      150
'''
observed_data = np.array([[60, 45, 30, 65],
                          [40, 30, 40, 40]])

# Step 3: Calculate Expected Frequencies
# Calculate expected frequencies using the chi-squared formula
row_totals = observed_data.sum(axis=1)
col_totals = observed_data.sum(axis=0)
total_obs = observed_data.sum()

expected_data = np.outer(row_totals, col_totals) / total_obs

# Step 4: Calculate the Chi-Squared Statistic
# Calculate the chi-squared statistic using the formula
chi_squared = np.sum((observed_data - expected_data)**2 / expected_data)

# Step 5: Determine Degrees of Freedom
# Calculate degrees of freedom (df)
df = (observed_data.shape[0] - 1) * (observed_data.shape[1] - 1)

# Step 6: Find Critical Value or P-Value
# Use the chi2_contingency function from scipy.stats to calculate the p-value
chi2_stat, p_value, _, _ = chi2_contingency(observed_data)

# Step 7: Compare Calculated Statistic with Critical Value or P-Value
alpha = 0.05

# Compare the calculated p-value with the chosen significance level
if p_value < alpha:
    conclusion = "Reject null hypothesis. There's a significant association."
else:
    conclusion = "Fail to reject null hypothesis. No significant association."

# Step 8: Interpret Results
print("Chi-Squared Statistic:", chi2_stat)
print("P-value:", p_value)
print("Degrees of Freedom:", df)
print(conclusion)
