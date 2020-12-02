#include "editor_action.h"

EditorAction::EditorAction(
    std::function<void(EditorParams)> action, 
    std::function<void(EditorParams)> undo_action, 
    EditorParams parameters):
    _action(action),
    _undo_action(undo_action),
    _params(parameters)
{

}

void EditorAction::undo()
{
    this->_undo_action(_params);
}

void EditorAction::redo()
{
    this->_action(_params);
}