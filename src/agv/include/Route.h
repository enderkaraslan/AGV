#pragma once

#include <queue>
#include <string>
#include <stdexcept>

class Route {
public:
    enum class TurnType {
        LEFT,
        RIGHT
    };

    struct Turn {
        TurnType type;
        int order;
    };

    Route()
    {
        addTurn(TurnType::LEFT,1);
        addTurn(TurnType::LEFT,1);
        addTurn(TurnType::LEFT,6);
        addTurn(TurnType::LEFT,1);
        addTurn(TurnType::LEFT,3);
    }
    ~Route() = default;

    bool isEmpty() const {
        return turns_.empty();
    }

    void addTurn(TurnType type, int order) {
        turns_.emplace(Turn{type, order});
    }

    Turn& getCurrentTurn() {
        if (isEmpty()) {
            throw std::out_of_range("No turns available");
        }
        return turns_.front();
    }

    TurnType& getCurrentTurnType() {
        return getCurrentTurn().type;
    }

    int& getCurrentTurnOrder() {
        return getCurrentTurn().order;
    }
    bool isTurn() const {
        if (!isEmpty() && turns_.front().order == 1) {
            return true;
        }
        return false;
    }
    void removeCurrentTurn() {
        if (isEmpty()) {
            throw std::out_of_range("No turns to remove");
        }
        turns_.pop();
    }

    size_t size() const {
        return turns_.size();
    }

private:
    std::queue<Turn> turns_;
};
