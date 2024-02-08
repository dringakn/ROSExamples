'''
Imagine you're investigating whether there's a connection between two things,
like the type of weather (sunny, cloudy, rainy) and people's preferences for
outdoor activities (playing sports, reading, watching movies).

You collect data from a bunch of people and make a table with rows for each
type of weather and columns for each activity.
Each cell in the table shows how many people like a specific activity in a
specific weather.

Now, you want to know: is there something going on, or are the preferences just random?
The chi-squared test helps you figure that out.

    Observed Counts:
        You start by looking at the actual counts in each cell of the table -
        how many people actually like a certain activity in a certain weather.

    Expected Counts:
        Then, you imagine a scenario where the type of weather and people's preferences are totally unrelated.
        You calculate what you would expect to see in each cell based on this assumption. This is like saying,
        "If there's no real connection, this is what we'd likely see."

    Comparing Observed and Expected:
        Now, you compare the actual counts you collected with the expected counts you calculated.
        If the numbers in the table are really close to the expected ones,
        it suggests that the two things (weather and preferences) probably don't have a strong connection.

    Calculation:
        You do some math to compare how much the observed counts differ from the expected ones in each cell.
        This gives you a single number called the "chi-squared statistic."

    Degrees of Freedom:
        This is like the "size" of your investigation.
        It depends on how big your table is.
        More rows and columns mean more degrees of freedom.
        It helps you figure out how significant your results are.

    P-Value:
        This is a fancy term for a probability.
        The p-value tells you how likely it is to see the differences you observed
        if there's actually no connection between weather and preferences.
        A small p-value means the differences are unlikely to be just random chance.

    Decision Time:
        If the p-value is really small (below a certain threshold you set beforehand),
        you might decide that the differences in the table are significant enough to say
        that there's likely a real connection between weather and preferences.
        You "reject the idea of no connection."

    Chi-Squared Distribution:
        Think of this like a reference chart.
        It tells you what to expect if there's really no connection between the two things you're studying.
        If your chi-squared statistic is way off from what the chart says, that's a sign that something interesting might be happening.

So, in simple terms, the chi-squared test helps you figure out if the relationship
between two categories of data is just by chance or if there's a meaningful connection.
It's like being a detective trying to find out if there's a hidden pattern in the
preferences of people based on different types of weather.
'''

from scipy.stats import chi2_contingency
import numpy as np
np.set_printoptions(precision=3, linewidth=120, suppress=False, sign='+')

# Create the observed data as a 2D array (contingency table)
# Null Hypothesis (H0): No connection between favorite color and transportation
# Alternative Hypothesis (Ha): Connection between favorite color and transportation
observed_data = np.array([
    [20, 15, 10],
    [25, 10, 5],
    [15, 5, 10],
    [10, 5, 5]
])

# Perform the chi-squared test
'''
Calculate expected frequencies using the chi-squared formula
row_totals = observed_data.sum(axis=1)
col_totals = observed_data.sum(axis=0)
total_obs = observed_data.sum()
expected = np.outer(row_totals, col_totals) / total_obs
chi_squared = np.sum((observed_data - expected_data)**2 / expected_data)
df = (observed_data.shape[0] - 1) * (observed_data.shape[1] - 1)
'''
chi2_stat, p_val, dof, expected = chi2_contingency(observed_data)

# Print the observed and expected counts
print(f"Observed Counts: \n{observed_data}")
print(f"Expected Counts: \n{expected}")

# Print the chi-squared statistic and p-value
print(f"\nChi-Squared Statistic:{chi2_stat:+0.3f}")
print(f"Degrees of Freedom:{dof:+0.3f}")
print(f"P-Value:{p_val:+0.3f}")

# Set the significance level (alpha)
alpha = 0.05  # significance level: 95%

# Last step: Interpret Results
# Compare the p-value with the significance level to make a decision
if p_val < alpha:
    '''
    If the calculated chi-squared statistic is greater than the critical value,
    you reject the null hypothesis and conclude that there's a significant connection
    between favorite color and transportation mode.
    '''
    print("\nReject the null hypothesis: There's evidence of a significant connection.")
else:
    '''
    If the p-value is less than your chosen significance level, you also reject the null hypothesis.
    '''
    print("\nFail to reject the null hypothesis: No significant connection.")
