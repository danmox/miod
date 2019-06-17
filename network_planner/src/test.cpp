#include <iostream>
#include <vector>

int int_pow(int x, int p)
{
  if (p == 0) return 1;
  if (p == 1) return x;

  int tmp = int_pow(x, p/2);
  if (p%2 == 0) return tmp * tmp;
  else return x * tmp * tmp;
}

std::vector<int> compute_combinations(const int len, const int num_digits)
{
  std::vector<int> index(len, 0);
  std::vector<int> combinations;

  while (true) {

    int perm = 0;
    for (int i = 0; i < len; ++i)
      perm += index[i] * int_pow(10,i);
    combinations.push_back(perm);

    for (int j = 0;; ++j) {
      if (j == len)
        return combinations;
      index[j]++;
      if (index[j] == num_digits)
        index[j] = 0;
      else
        break;
    }

  }
}

std::vector<int> extract_inds(int num, const int agents)
{
  std::vector<int> inds(agents,0);
  for (int i = agents-1; i >= 0; --i) {
    int dec = int_pow(10,i);
    inds[i] = num / dec;
    num = num % dec;
  }
  return inds;
}

int main(int argc, char** argv)
{
  int digits = 8; // i.e. each digit can be drawn from {0,1,2,3}
  int agents = 4; // want index sets of length 2

  std::vector<int> combinations = compute_combinations(agents, digits);

  /*
  for (int num : combinations) {
    std::vector<int> inds = extract_inds(num, agents);
    for (int ind : inds)
      printf("%d ", ind);
    printf("\n");
  }
  */
  printf("%ld combinations\n", combinations.size());

  return 0;
}
