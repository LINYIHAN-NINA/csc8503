#include "HighScoreTable.h"

#include <fstream>
#include <algorithm>
#include <sstream>

using namespace NCL::CSC8503;

namespace
{
    constexpr std::size_t kDefaultMaxScores = 10;
}

bool HighScoreTable::Load(const std::string &path)
{
    entries.clear();
    std::ifstream input(path);
    if (!input.is_open())
    {
        return false;
    }

    std::string line;
    while (std::getline(input, line))
    {
        if (line.empty())
        {
            continue;
        }
        std::istringstream lineStream(line);
        HighScoreEntry entry;
        lineStream >> std::ws;
        std::getline(lineStream, entry.name, ',');
        lineStream >> entry.score;
        if (!entry.name.empty())
        {
            entries.emplace_back(entry);
        }
    }
    input.close();
    std::sort(entries.begin(), entries.end(), [](const HighScoreEntry &a, const HighScoreEntry &b)
              { return a.score > b.score; });
    if (entries.size() > kDefaultMaxScores)
    {
        entries.resize(kDefaultMaxScores);
    }
    return true;
}

bool HighScoreTable::Save(const std::string &path) const
{
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open())
    {
        return false;
    }
    for (const auto &entry : entries)
    {
        output << entry.name << "," << entry.score << "\n";
    }
    return true;
}

void HighScoreTable::AddScore(const HighScoreEntry &entry, std::size_t maxEntries)
{
    entries.emplace_back(entry);
    std::sort(entries.begin(), entries.end(), [](const HighScoreEntry &a, const HighScoreEntry &b)
              { return a.score > b.score; });
    if (entries.size() > maxEntries)
    {
        entries.resize(maxEntries);
    }
}

void HighScoreTable::Clear()
{
    entries.clear();
}
