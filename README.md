# Path Finding Algorithm

This project implements several path-finding algorithms: **BFS**, **DFS**, **UCS**, and **A\***.

## 🛠️ How to Use

1. **Download the code** to your local machine.
2. Open your terminal in the project directory.
3. Compile the C++ code using:

```bash
g++ PathSearch.cpp -o PathSearch
```

3. **Run the program using the format:**

```bash
./PathSearch <input file name> <search algo (bfs/dfs/ucs/a*)> <source> <destination> *<heuristic file>
```

### Example:

```bash
./PathSearch input.txt bfs Omaha Dallas
```

If you are using **A\*** Search, you **must** specify the heuristic file:

```bash
./PathSearch input.txt a* Omaha Dallas heuristic.txt
```

---

## Input File Format

Each line in the **input file** should be in the format:

```
<Source> <Destination> <Path Cost>
```

Example:
```
Chicago Milwaukee 93
Milwaukee Madison 79
Milwaukee DesMoines 269
```

---

## Heuristic File Format (Only for A*)

Each line should contain the city and the estimated cost to the destination:

```
<City> <Heuristic Cost>
```

Example:
```
Chicago 750
Milwaukee 820
Madison 790
```

---

## Supported Algorithms

- **BFS** – Breadth-First Search
- **DFS** – Depth-First Search
- **UCS** – Uniform Cost Search
- **A\*** – A-Star Search (requires heuristic file)

---

## Notes

- Ensure the heuristic file is only used when running A* search.
- File names must match exactly with your local files.
- Cities and search types are case-sensitive.

---
