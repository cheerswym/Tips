call plug#begin('~/.vim/bundle')
call plug#end()

call plug#begin()
Plug 'preservim/nerdtree'
Plug 'Yggdroot/LeaderF', { 'do': './install.sh' }
Plug 'Valloric/YouCompleteMe'
call plug#end()
map <C-n> :NERDTreeToggle<CR>

highlight PMenu ctermfg=0 ctermbg=242 guifg=white guibg=white
highlight PMenuSel ctermfg=242 ctermbg=8 guifg=white guibg=white

syntax on
colorscheme desert
set showmatch
set nocompatible
set incsearch
set clipboard=unnamed
set guicursor=i:block-iCursor-blinkon0,v:block-vCursor
map <F4> :e %:p:s,.h$,.X123X,:s,.cc$,.h,:s,.X123X$,.cc,<CR>
set nrformats=
set history=200
