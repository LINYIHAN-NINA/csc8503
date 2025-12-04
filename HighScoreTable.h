#pragma once
#include <vector>
#include <string>

namespace NCL::CSC8503
{
    struct HighScoreEntry
    {
        std::string name;
        int score = 0;
    };

    class HighScoreTable
    {
    public:
        bool Load(const std::string &path);
        bool Save(const std::string &path) const;

        void AddScore(const HighScoreEntry &entry, std::size_t maxEntries = 10);
        void Clear();

        const std::vector<HighScoreEntry> &GetEntries() const { return entries; }

    private:
        std::vector<HighScoreEntry> entries;
    };
}
